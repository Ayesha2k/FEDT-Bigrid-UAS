#pragma once
#include <rclcpp/rclcpp.hpp>
#include <crazyflie_interfaces/msg/position.hpp>
#include <crazyflie_interfaces/msg/full_state.hpp>
#include <crazyflie_interfaces/msg/hover.hpp>
#include <crazyflie_interfaces/msg/velocity_world.hpp>
#include <crazyflie_interfaces/srv/notify_setpoints_stop.hpp>
#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"
class GenericCommander {
 public:
  GenericCommander(std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface>,
    std::shared_ptr<WebotsCrazyflieDriver>);
 private:
  rclcpp::CallbackGroup::SharedPtr callbacks_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<crazyflie_interfaces::srv::NotifySetpointsStop>::SharedPtr release_;
};
