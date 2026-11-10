#pragma once

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/empty.hpp"

class Parameters
{
public:
    Parameters(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface,
        std::shared_ptr<WebotsCrazyflieDriver> webots_driver
    );
    void initialize_parameters();

private:
    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult set_parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

private:
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_logging_interface;    

    std::weak_ptr<WebotsCrazyflieDriver> m_webots_driver;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_downdload_toc_sub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> m_get_toc_info_sub;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle;
};