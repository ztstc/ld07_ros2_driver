#pragma once

#include "ld07_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

class LD07Node : public rclcpp::Node {
public:
  LD07Node();

private:
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;

  LD07Driver driver_;

  bool enable_scan_;
  bool enable_cloud_;
  bool autostart_;

  void startCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>);
  void stopCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>);
};
