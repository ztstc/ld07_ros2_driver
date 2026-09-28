#include "ld07_ros2_driver/ld07_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>

LD07Node::LD07Node()
    : Node("ld07_node"), driver_("/dev/ttyUSB0", this->get_logger()) {
  declare_parameter("enable_scan", true);
  declare_parameter("enable_cloud", true);
  declare_parameter("autostart", true);

  enable_scan_ = get_parameter("enable_scan").as_bool();
  enable_cloud_ = get_parameter("enable_cloud").as_bool();
  autostart_ = get_parameter("autostart").as_bool();

  if (enable_scan_)
    laser_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  if (enable_cloud_)
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(20),
                             std::bind(&LD07Node::timerCallback, this));

  start_service_ = create_service<std_srvs::srv::Empty>(
      "start", std::bind(&LD07Node::startCallback, this, std::placeholders::_1,
                         std::placeholders::_2));
  stop_service_ = create_service<std_srvs::srv::Empty>(
      "stop", std::bind(&LD07Node::stopCallback, this, std::placeholders::_1,
                        std::placeholders::_2));

  driver_.stop();
  if (autostart_)
    driver_.start();
}

void LD07Node::timerCallback() {
  // RCLCPP_WARN(this->get_logger(), "timerCallback");
  if (!driver_.isRunning())
  {
    return;
  }
    // RCLCPP_WARN(this->get_logger(), "timerCallback running");

  std::vector<uint8_t> frame;
  if (!driver_.readFrame(frame))
  {
    // RCLCPP_WARN(this->get_logger(), "readFrame failed");
    return;
  }
  // RCLCPP_INFO(this->get_logger(), "frame size: %zu", frame.size());

  sensor_msgs::msg::LaserScan scan;
  sensor_msgs::msg::PointCloud2 cloud;
  // RCLCPP_INFO(this->get_logger(), "time for parseData");
  if (!driver_.parseData(frame, scan, cloud)) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse data frame");
    return;
  }
  // RCLCPP_WARN(this->get_logger(), "time for publishing");
  scan.header.stamp = this->now();
  cloud.header.stamp = this->now();
  // if (enable_scan_ && laser_pub_) {
  //   laser_pub_->publish(scan);
  // }
  if (enable_cloud_ && cloud_pub_) {
    cloud_pub_->publish(cloud);
  }
}

void LD07Node::startCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>) {
  driver_.start();
}

void LD07Node::stopCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>) {
  driver_.stop();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LD07Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}