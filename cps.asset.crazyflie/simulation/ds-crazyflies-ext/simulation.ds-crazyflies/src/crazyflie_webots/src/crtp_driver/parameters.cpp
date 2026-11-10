#include "crazyflie_webots/crtp_driver/parameters.hpp"
using std::placeholders::_1;

Parameters::Parameters(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_interface,
    std::shared_ptr<WebotsCrazyflieDriver> webots_driver)
: m_logging_interface(node_logging_interface)
, m_webots_driver(webots_driver)
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
    (void)node_parameters_interface;
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_downdload_toc_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/download_parameters_toc",
        rclcpp::QoS(10),
        std::bind(&Parameters::download_toc_callback, this, _1),
        sub_opt);

    m_get_toc_info_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        "~/get_parameters_toc_info",
        10,
        std::bind(&Parameters::get_toc_info_callback, this, _1),
        sub_opt);

    RCLCPP_DEBUG(node_logging_interface->get_logger(), "Parameters  initialized");
}

void Parameters::initialize_parameters()
{
    
}

rcl_interfaces::msg::SetParametersResult Parameters::set_parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    // This gets called if a parameter gets set. We want to set it on the crazyflie as well.

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
        std::string param_name = param.get_name();
        size_t dot = param_name.find('.');

        if (dot != std::string::npos)
        {
            std::string group = param_name.substr(0, dot);
            std::string name = param_name.substr(dot + 1);

            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                //
            }
            else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                //
            }
        }

        if (!result.successful)
            return result;
    }
    return result;
}

void Parameters::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(m_logging_interface->get_logger(), "Downloading parameters TOC");
}

void Parameters::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(m_logging_interface->get_logger(), "Getting parameters TOC info");
}