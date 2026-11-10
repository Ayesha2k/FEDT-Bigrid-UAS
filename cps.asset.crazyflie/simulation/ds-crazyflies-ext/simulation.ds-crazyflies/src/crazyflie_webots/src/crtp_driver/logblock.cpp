#include "crazyflie_webots/crtp_driver/logblock.hpp"

using std::placeholders::_1;


LogBlock::LogBlock( 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group,
    const std::string name,
    GetDataCallback get_data_callback
)
: m_base_interface(node_base_interface)
, m_logging_interface(node_logging_interface)
, m_timers_interface(node_timers_interface)
, m_clock_interface(node_clock_interface)
, m_callback_group(callback_group)
, m_block_name(name)
, m_get_data_callback(get_data_callback)
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = callback_group;
    m_log_data_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(
        node_topics_interface,
        "~/" + name,
        rclcpp::QoS(10),
        publisher_options
    );       
}

void LogBlock::start_log_block(int period_ms)
{
    m_publish_log_data_timer = rclcpp::create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&LogBlock::m_publish_log_data, this),
        m_callback_group,
        m_base_interface.get(),
        m_timers_interface.get()
    );
    RCLCPP_INFO(m_logging_interface->get_logger(), "Starting log block with period_ms: %d", period_ms);
}

void LogBlock::stop_log_block()
{
    m_publish_log_data_timer->cancel();
    m_publish_log_data_timer.reset();
    RCLCPP_INFO(m_logging_interface->get_logger(), "Stopping log block");
}

void LogBlock::m_publish_log_data()
{
    auto log_msg = crazyflie_interfaces::msg::LogDataGeneric();
    if (m_get_data_callback) {
        auto values =  m_get_data_callback(m_block_name);
        for (const auto &val : values) {
            log_msg.values.push_back(val);
        }
    }
    log_msg.header.stamp = m_clock_interface->get_clock()->now();
    m_log_data_publisher->publish(log_msg);
}