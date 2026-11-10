#pragma once

#include <map>

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"
#include "crazyflie_webots/crtp_driver/logblock.hpp"

#include "crazyflie_interfaces/msg/log_data_generic.hpp"

#include "crazyflie_interfaces/srv/add_logging.hpp"
#include "crazyflie_interfaces/srv/remove_logging.hpp"

#include "std_msgs/msg/empty.hpp"



class Logging{
public:
    Logging(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<WebotsCrazyflieDriver> webots_driver
    );

private: 

    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void m_add_log_block_service(
        const std::shared_ptr<crazyflie_interfaces::srv::AddLogging::Request> request,
        std::shared_ptr<crazyflie_interfaces::srv::AddLogging::Response> response
    );

    void m_remove_log_block_service(
        const std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging::Request> request,
        std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging::Response> response
    );

    bool m_create_log_block(
        const std::string &block_name,
        const std::vector<std::string> &variables);


    std::vector<double> m_get_data_callback(const std::string &block_name);

    void publish_state_timer_callback();

private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> m_topics_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_timers_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_clock_interface;

    std::weak_ptr<WebotsCrazyflieDriver> m_webots_driver;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_downdload_toc_sub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_get_toc_info_sub;

    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::AddLogging>> m_add_log_block_server;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::RemoveLogging>> m_remove_log_block_server;

    std::shared_ptr<rclcpp::TimerBase> m_publish_state_timer;
    std::shared_ptr<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>> m_state_publisher;

    std::map<std::string, std::shared_ptr<LogBlock>> m_log_blocks;
    std::map<std::string, std::vector<std::string>> m_log_block_variables;

    const std::map<std::string, std::function<double(std::shared_ptr<WebotsCrazyflieDriver>)>> kVariableMap = {
        {"range.front",      [](auto d) { return d->get_range_front(); }},
        {"range.back",       [](auto d) { return d->get_range_back(); }},
        {"range.up",         [](auto d) { return d->get_range_up(); }},
        {"range.left",       [](auto d) { return d->get_range_left(); }},
        {"range.right",      [](auto d) { return d->get_range_right(); }},
        {"range.zrange",     [](auto d) { return d->get_range_zrange(); }},
    
        {"stateEstimate.x",  [](auto d) { return d->get_robot_pose().translation().x(); }},
        {"stateEstimate.y",  [](auto d) { return d->get_robot_pose().translation().y(); }},
        {"stateEstimate.z",  [](auto d) { return d->get_robot_pose().translation().z(); }},
        {"stateEstimate.yaw",[](auto d) {
            Eigen::Affine3d pose = d->get_robot_pose();
            Eigen::Matrix3d R = pose.rotation();
            return atan2(R(1,0), R(0,0));
        }},
    
        {"stateEstimate.qx", [](auto d) { return Eigen::Quaterniond(d->get_robot_pose().rotation()).x(); }},
        {"stateEstimate.qy", [](auto d) { return Eigen::Quaterniond(d->get_robot_pose().rotation()).y(); }},
        {"stateEstimate.qz", [](auto d) { return Eigen::Quaterniond(d->get_robot_pose().rotation()).z(); }},
        {"stateEstimate.qw", [](auto d) { return Eigen::Quaterniond(d->get_robot_pose().rotation()).w(); }},
    
        {"pm.vbat",          [](auto d) { return d->get_battery_voltage(); }},
        {"pm.chargeCurrent", [](auto d) { return d->get_charge_current(); }},
        {"pm.state",         [](auto d) { return d->get_charge_state(); }}
    };
};  