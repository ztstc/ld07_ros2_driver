#pragma once

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>


struct Point {
  float x, y;
};


class LD07Driver {
public:
  LD07Driver(const std::string &port, rclcpp::Logger logger);
  bool start();
  void stop();
  bool isRunning() const;
  bool readFrame(std::vector<uint8_t> &frame);
  bool parseData(const std::vector<uint8_t> &frame,
                 sensor_msgs::msg::LaserScan &scan,
                 sensor_msgs::msg::PointCloud2 &cloud);

private:
  std::string port_;
  int fd_ = -1;
  bool running_ = false;
  rclcpp::Logger logger_;
  std::vector<double> k_, b_;
  int points_per_camera_ = 80;

  std::atomic<bool> stop_thread_;
  std::thread read_thread_;
  std::mutex buffer_mutex_;
  std::deque<uint8_t> buffer_;

  bool configureSerialPort();
  bool sendCommand(uint8_t cmd, uint8_t address = 0x01);
  bool getCorrectionParams();
  void serialReadThread();
  void angTransform(uint16_t dist, int n, double &theta, double &r);
  std::vector<Point> correctXY(const std::vector<uint16_t>& rawDistances);

  // The CH340 adapter may be re-enumerated (ttyUSB0 -> ttyUSB1) or drop out
  // when the supply browns out; recover the link instead of going silent.
  static std::string findSerialPort();
  bool reconnect();
};