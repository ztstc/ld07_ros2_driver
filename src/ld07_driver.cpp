#include "ld07_ros2_driver/ld07_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <deque>
#include <thread>
#include <atomic>
#include <cerrno>
#include <glob.h>

LD07Driver::LD07Driver(const std::string &port, rclcpp::Logger logger)
    : port_(port), logger_(logger), running_(false), stop_thread_(false) {}

std::string LD07Driver::findSerialPort() {
  glob_t g;
  std::string found;
  if (glob("/dev/ttyUSB*", 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
    found = g.gl_pathv[0];
  }
  globfree(&g);
  return found;
}

bool LD07Driver::configureSerialPort() {
  // Re-discover the port: the adapter may come back as ttyUSB1, ttyUSB2, ...
  std::string candidate = findSerialPort();
  if (!candidate.empty()) port_ = candidate;

  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open port %s", port_.c_str());
    return false;
  }

  termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Error getting serial attributes");
    return false;
  }

  cfsetospeed(&tty, B921600);
  cfsetispeed(&tty, B921600);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 5;

  tcflush(fd_, TCIFLUSH);
  return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

void LD07Driver::serialReadThread() {
  uint8_t temp[256];
  int consecutive_errors = 0;
  auto last_data = std::chrono::steady_clock::now();
  while (!stop_thread_) {
    int bytes_read = read(fd_, temp, sizeof(temp));
    if (bytes_read > 0) {
      consecutive_errors = 0;
      last_data = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      buffer_.insert(buffer_.end(), temp, temp + bytes_read);
    } else {
      // read() == 0 means EOF: the tty was unplugged / re-enumerated.
      // read() < 0 with EIO/ENODEV means the same. Both need a reconnect.
      bool fatal = (bytes_read == 0) ||
                   (bytes_read < 0 && errno != EAGAIN && errno != EINTR);
      if (fatal) {
        if (++consecutive_errors >= 3) {
          RCLCPP_WARN(logger_, "Serial link lost; attempting reconnect");
          consecutive_errors = 0;
          if (!reconnect()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          } else {
            last_data = std::chrono::steady_clock::now();
          }
        }
      } else if (std::chrono::steady_clock::now() - last_data >
                 std::chrono::seconds(2)) {
        // Data stopped arriving without an error: the device browned out.
        RCLCPP_WARN(logger_, "No serial data for 2s; attempting reconnect");
        last_data = std::chrono::steady_clock::now();
        if (!reconnect()) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
          last_data = std::chrono::steady_clock::now();
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool LD07Driver::reconnect() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
  }

  if (!configureSerialPort()) return false;
  if (!getCorrectionParams()) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
  }
  if (!sendCommand(0x02)) return false;
  RCLCPP_INFO(logger_, "Reconnected to %s", port_.c_str());
  return true;
}

bool LD07Driver::sendCommand(uint8_t cmd, uint8_t address) {
  uint8_t packet[11] = {0xAA, 0xAA, 0xAA, 0xAA,
                        address, cmd, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t checksum = 0;
  for (int i = 4; i < 10; ++i) checksum += packet[i];
  packet[10] = checksum;
  return write(fd_, packet, 11) == 11;
}

bool LD07Driver::getCorrectionParams() {
  RCLCPP_INFO(logger_, "getCorrectionParams");

  // The sensor may still be streaming from a previous session; stop it and drop
  // any stale bytes so the answer packet is not interleaved with scan data.
  sendCommand(0x0F);
  usleep(50000);
  tcflush(fd_, TCIFLUSH);

  sendCommand(0x12);

  // Answer layout: AAAAAAAA addr cmd offset(2) len(2) + 18 data bytes + checksum
  constexpr uint16_t kCoeDataLen = 18;
  constexpr size_t kCoePacketLen = 10 + kCoeDataLen + 1;

  std::vector<uint8_t> rx;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline &&
         rx.size() < kCoePacketLen) {
    uint8_t temp[64];
    int n = read(fd_, temp, sizeof(temp));
    if (n > 0) {
      rx.insert(rx.end(), temp, temp + n);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Locate the answer frame (cmd 0x12) and validate its checksum.
  const uint8_t *pkt = nullptr;
  for (size_t i = 0; i + kCoePacketLen <= rx.size(); ++i) {
    if (rx[i] != 0xAA || rx[i + 1] != 0xAA || rx[i + 2] != 0xAA ||
        rx[i + 3] != 0xAA || rx[i + 5] != 0x12) {
      continue;
    }
    uint16_t len = rx[i + 8] | (rx[i + 9] << 8);
    size_t total = 10 + len + 1;
    if (len != kCoeDataLen || i + total > rx.size()) continue;
    uint8_t sum = 0;
    for (size_t k = i + 4; k < i + total - 1; ++k) sum += rx[k];
    if (sum == rx[i + total - 1]) {
      pkt = &rx[i];
      break;
    }
  }

  if (!pkt) {
    RCLCPP_ERROR(logger_,
                 "No valid correction-parameter answer (cmd 0x12) received");
    return false;
  }

  const uint8_t *d = pkt + 10;  // payload start
  auto le32 = [](const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  };

  k_.resize(2);
  b_.resize(2);
  k_[0] = static_cast<double>(le32(d + 0)) / 10000.0;
  k_[1] = static_cast<double>(le32(d + 4)) / 10000.0;
  b_[0] = static_cast<double>(le32(d + 8)) / 10000.0;
  b_[1] = static_cast<double>(le32(d + 12)) / 10000.0;
  points_per_camera_ = d[16] | (d[17] << 8);

  if (points_per_camera_ <= 0 || points_per_camera_ > 1024) {
    RCLCPP_ERROR(logger_, "Invalid points per camera: %d", points_per_camera_);
    return false;
  }
  RCLCPP_INFO(logger_, "getCorrectionParams Success");
  RCLCPP_INFO(logger_, "k[0]: %f, k[1]: %f", k_[0], k_[1]);
  RCLCPP_INFO(logger_, "b[0]: %f, b[1]: %f", b_[0], b_[1]);
  RCLCPP_INFO(logger_, "points_per_camera_: %d", points_per_camera_);
  return true;
}

bool LD07Driver::start() {
  RCLCPP_INFO(logger_, "Starting LD07 driver");
  if (!configureSerialPort()) return false;
  if (!getCorrectionParams()) return false;
  {
    // Drop any stale bytes so the first frame starts on a real header.
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.clear();
  }
  RCLCPP_INFO(logger_, "Send command to start");
  sendCommand(0x02);
  running_ = true;
  stop_thread_ = false;
  read_thread_ = std::thread(&LD07Driver::serialReadThread, this);
  return true;
}

void LD07Driver::stop() {
  RCLCPP_INFO(logger_, "Stopping LD07 driver");
  sendCommand(0x0F);
  running_ = false;
  stop_thread_ = true;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  close(fd_);
}

bool LD07Driver::isRunning() const {
  return running_;
}

bool LD07Driver::readFrame(std::vector<uint8_t> &frame) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  while (buffer_.size() >= 10) {
    // Check for header
    if (buffer_[0] == 0xAA && buffer_[1] == 0xAA && buffer_[2] == 0xAA && buffer_[3] == 0xAA) {
      int length = buffer_[8] + (buffer_[9] << 8);
      // RCLCPP_INFO(logger_, "Found header, Frame length: %d", length);
      if (buffer_.size() >= 11 + length) {
        // RCLCPP_INFO(logger_, "msg ready");
        frame.assign(buffer_.begin() + 10, buffer_.begin() + 11 + length);
        buffer_.erase(buffer_.begin(), buffer_.begin() + 11 + length);
        return true;
      } else {
        // Wait for more data
        break;
      }
    } else {
      // Remove invalid data
      buffer_.pop_front();
      RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 2000,
                           "Dropping out-of-sync serial byte");
    }
  }

  return false;
}



void LD07Driver::angTransform(uint16_t dist, int n, double &theta, double &r) {
  const double pi = 3.14159265;
  double pixelU = n;
  double tempTheta, tempDist, tempX, tempY;

  if (n >= 80) {
      pixelU = 160 - pixelU;
      tempTheta = b_[0] > 1 ? (k_[0] * pixelU - b_[0]) : atan(k_[0] * pixelU - b_[0]) * 180 / pi;
      tempDist = (dist - 1.22) / cos((22.5 - tempTheta) * pi / 180);
      tempTheta *= pi / 180;
      tempX =  cos(22.5 * pi / 180) * tempDist * cos(tempTheta) + sin(22.5 * pi / 180) * tempDist * sin(tempTheta);
      tempY = -sin(22.5 * pi / 180) * tempDist * cos(tempTheta) + cos(22.5 * pi / 180) * tempDist * sin(tempTheta);
      tempX += 1.22;
      tempY -= 5.315;
  } else {
      pixelU = 80 - pixelU;
      tempTheta = b_[1] > 1 ? (k_[1] * pixelU - b_[1]) : atan(k_[1] * pixelU - b_[1]) * 180 / pi;
      tempDist = (dist - 1.22) / cos((22.5 + tempTheta) * pi / 180);
      tempTheta *= pi / 180;
      tempX =  cos(-22.5 * pi / 180) * tempDist * cos(tempTheta) + sin(-22.5 * pi / 180) * tempDist * sin(tempTheta);
      tempY = -sin(-22.5 * pi / 180) * tempDist * cos(tempTheta) + cos(-22.5 * pi / 180) * tempDist * sin(tempTheta);
      tempX += 1.22;
      tempY += 5.315;
  }
  r = sqrt(tempX * tempX + tempY * tempY) * 0.001;
  theta = atan2(tempY, tempX);
}

std::vector<Point> LD07Driver::correctXY(const std::vector<uint16_t>& rawDistances) {
  // Keep one entry per measurement index so the output stays aligned with the
  // point-cloud width. Invalid returns (distance == 0) become (0, 0).
  std::vector<Point> points;
  points.reserve(rawDistances.size());
  for (int i = 0; i < static_cast<int>(rawDistances.size()); ++i) {
      if (rawDistances[i] == 0) {
        points.push_back({0.0f, 0.0f});
        continue;
      }
      double theta, r;
      angTransform(rawDistances[i], i, theta, r);
      points.push_back({static_cast<float>(r * cos(theta)), static_cast<float>(r * sin(theta))});
  }
  return points;
}


bool LD07Driver::parseData(const std::vector<uint8_t> &frame,
                           sensor_msgs::msg::LaserScan &scan,
                           sensor_msgs::msg::PointCloud2 &cloud) {
  // scan.header.frame_id = "ld07_frame";
  // scan.angle_min = 0.0;
  // scan.angle_max = 2 * M_PI;
  // scan.angle_increment = (scan.angle_max - scan.angle_min) / (points_per_camera_ * 2);
  // scan.time_increment = 0.0;
  // scan.range_min = 0.03;
  // scan.range_max = 3.0;
  // scan.ranges.resize(points_per_camera_ * 2);
  // scan.intensities.resize(points_per_camera_ * 2);

  cloud.header.frame_id = "ld07_frame";
  cloud.height = 1;
  cloud.width = points_per_camera_ * 2;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  cloud.data.resize(cloud.row_step * cloud.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  
  if( frame.size() < 4 + points_per_camera_ * 2 * 2) {
    RCLCPP_ERROR(logger_, "Frame size is too small");
    return false;
  }

  std::vector<uint16_t> rawDistances;
  std::vector<uint16_t> rawQuality;
  uint32_t timeStamp = *(uint32_t*)&frame[0];
  for (int i = 0; i < points_per_camera_ * 2; ++i) {
    uint16_t raw = *(uint16_t*)&frame[4 + 2 * i];
    // uint16_t raw = frame[4 + 2 * i] + (frame[5 + 2 * i] << 8);
    rawDistances.push_back(raw&0x1FF);
    rawQuality.push_back((raw >> 9) & 0x7F);
  }
  std::vector<Point> points = correctXY(rawDistances);
  for (int i = 0; i < points.size(); ++i) {
    // RCLCPP_INFO(logger_, "x: %f, y: %f", points[i].x, points[i].y);
    // *iter_x = rawDistances[i]*0.001;
    // *iter_y = 0.001 * i;
    // *iter_z = rawQuality[i]*0.001;
    *iter_x = points[i].x;
    *iter_y = points[i].y;
    *iter_z = 0.0;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return true;
}