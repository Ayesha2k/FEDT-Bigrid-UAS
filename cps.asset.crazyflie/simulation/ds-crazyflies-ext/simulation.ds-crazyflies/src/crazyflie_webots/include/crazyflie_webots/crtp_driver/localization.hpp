#pragma once
#include <rclcpp/rclcpp.hpp>
#include <crazyflie_interfaces/msg/pose_named_array.hpp>
#include <crazyflie_interfaces/msg/status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"
class Localization {
 public:
  Localization(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>,
    std::shared_ptr<WebotsCrazyflieDriver>);
 private:
  rclcpp::Publisher<crazyflie_interfaces::msg::PoseNamedArray>::SharedPtr pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Publisher<crazyflie_interfaces::msg::Status>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_;
  rclcpp::CallbackGroup::SharedPtr callbacks_;
  rclcpp::TimerBase::SharedPtr timer_;
};
