#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <crazyflie_interfaces/srv/takeoff.hpp>
#include <crazyflie_interfaces/srv/land.hpp>
#include <crazyflie_interfaces/srv/go_to.hpp>
#include <crazyflie_interfaces/srv/start_trajectory.hpp>
#include <crazyflie_interfaces/srv/upload_trajectory.hpp>
#include <crazyflie_interfaces/srv/set_group_mask.hpp>
#include <crazyflie_interfaces/srv/stop.hpp>
#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"

class HighLevelCommander {
 public:
  HighLevelCommander(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface>,
    std::shared_ptr<WebotsCrazyflieDriver>);
 private:
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::CallbackGroup::SharedPtr callbacks_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr schedule_;
};
