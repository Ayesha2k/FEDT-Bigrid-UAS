#include "crazyflie_webots_gateway/crazyflie_lifecycle_client.hpp"


CrazyflieLifecycleClient::CrazyflieLifecycleClient(
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    rclcpp::CallbackGroup::SharedPtr callback_group,
    int id,
    DisconnectCallback disconnect_callback)
    : m_id(id)
    , m_disconnect_callback(disconnect_callback)
{
    auto subscribe_options = rclcpp::SubscriptionOptions();
    subscribe_options.callback_group = callback_group;

    std::string prefix = "/cf" + std::to_string(id);

    m_transition_event_sub = rclcpp::create_subscription<lifecycle_msgs::msg::TransitionEvent>(
        node_topics_interface,
        prefix + "/transition_event",
        rclcpp::QoS(10),
        std::bind(&CrazyflieLifecycleClient::m_transition_event_callback, this, std::placeholders::_1),
        subscribe_options);

    m_change_state_client = rclcpp::create_client<lifecycle_msgs::srv::ChangeState>(
        node_base_interface,
        node_graph_interface,
        node_services_interface,
        prefix + "/change_state",
        rmw_qos_profile_services_default,
        callback_group);    
}

CrazyflieLifecycleClient::~CrazyflieLifecycleClient()
{
    m_transition_event_sub.reset();
    m_change_state_client.reset();
}
bool 
CrazyflieLifecycleClient::wait_for_change_state_service(std::chrono::milliseconds timeout_ms)
{
    return m_change_state_client->wait_for_service(timeout_ms);
}


std::future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>>
CrazyflieLifecycleClient::transition_crazyflie_async(uint8_t id, const std::string &label)
{
    // Wait/Check/Ensure that the service (and this client) is available

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = id;
    request->transition.label = label;
    auto res =  m_change_state_client->async_send_request(request);
    // res is a FutureAndRequestId; return the inner std::future for the response
    return std::move(res.future);
}

bool 
CrazyflieLifecycleClient::transition_crazyflie_sync(uint8_t id, const std::string &label, std::optional<std::chrono::milliseconds> timeout_ms)
{
    if (timeout_ms.has_value())
    {
        std::chrono::time_point<std::chrono::steady_clock> start_time =  std::chrono::steady_clock::now();
        m_change_state_client->wait_for_service(timeout_ms.value());
        std::chrono::time_point<std::chrono::steady_clock> end_time =  std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        if (elapsed >= timeout_ms.value())
        {
            std::cerr << "Timeout while waiting for change_state service of crazyflie " << m_id << std::endl;
            return false;
        }
        timeout_ms.value() -= elapsed;
    } else {
        m_change_state_client->wait_for_service();
    }



    auto future = transition_crazyflie_async(id, label);
    if (timeout_ms.has_value())
    {
        auto status = future.wait_for(timeout_ms.value());
        if (status == std::future_status::ready) return future.get()->success;
        else return false;
    }
    else
    {
        future.wait();
        return future.get()->success;
    }
}

bool 
CrazyflieLifecycleClient::activate_crazyflie_sync(std::chrono::milliseconds timeout_ms)
{
   return transition_crazyflie_sync(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, "activate",timeout_ms);
}

bool 
CrazyflieLifecycleClient::configure_crazyflie_sync(std::chrono::milliseconds timeout_ms)
{
   return transition_crazyflie_sync(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING, "configure", timeout_ms);
}

bool 
CrazyflieLifecycleClient::shutdown_crazyflie_sync(std::chrono::milliseconds timeout_ms)
{
    try {
        return transition_crazyflie_sync(lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN, "shutdown", timeout_ms);
    } catch (const std::future_error &e) {
        if (e.code() == std::make_error_code(std::future_errc::broken_promise)) {
            // This is okay because it means the crazyflie node has already shut down
            return true;
        } else {
            std::cerr << "Caught future_error: " << e.what() << std::endl;
            return false;
        }
    }
}

std::future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>> 
CrazyflieLifecycleClient::configure_crazyflie_async()
{
    return transition_crazyflie_async(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING, "configure");
}

std::future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>> 
CrazyflieLifecycleClient::activate_crazyflie_async()
{
    return transition_crazyflie_async(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING, "activate");
}

std::future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>> 
CrazyflieLifecycleClient::deactivate_crazyflie_async()
{
    return transition_crazyflie_async(lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING, "deactivate");
}

std::future<std::shared_ptr<lifecycle_msgs::srv::ChangeState::Response>> 
CrazyflieLifecycleClient::shutdown_crazyflie_async()
{
    return transition_crazyflie_async(lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN, "shutdown");
}



void 
CrazyflieLifecycleClient::m_transition_event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
    if (msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
    {
        if (m_disconnect_callback) m_disconnect_callback(m_id);
    }
}