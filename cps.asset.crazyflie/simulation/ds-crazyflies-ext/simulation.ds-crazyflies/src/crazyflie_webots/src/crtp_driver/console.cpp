#include "crazyflie_webots/crtp_driver/console.hpp"
using std::placeholders::_1;

Console::Console(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)
: m_logging_interface(node_logging_interface)
, m_webots_driver(webots_driver)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = m_callback_group;

    m_console_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
        node_topics_interface,
        "~/console",
        rclcpp::QoS(10),
        publisher_options);

    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Console initialized");
}

void Console::console_message(const std::string message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    m_console_publisher->publish(msg);
}