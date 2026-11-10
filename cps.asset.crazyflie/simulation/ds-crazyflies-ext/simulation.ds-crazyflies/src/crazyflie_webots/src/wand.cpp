#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "crazyflie_webots/webots_driver/webots_wand_driver.hpp"

#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/qos.hpp"


class Wand : public rclcpp_lifecycle::LifecycleNode
{
  public: 
    Wand(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("wand", options)
    , m_id(declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_webots_port(declare_parameter("webots_port", rclcpp::ParameterValue(1234), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_webots_use_tcp(declare_parameter("webots_use_tcp", rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<bool>())
    , m_webots_tcp_ip(declare_parameter("webots_tcp_ip", rclcpp::ParameterValue("127.0.0.1"), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>())
    , m_wb_driver(std::make_shared<WebotsWandDriver>(m_id, m_webots_port, m_webots_use_tcp, m_webots_tcp_ip))
    {
      // We would like to use tf2_ros::TransformBroadcaster but it seems to have issues with lifecycle nodes -> crashes
      m_tf_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto publisher_options = rclcpp::PublisherOptions();
      publisher_options.callback_group = m_tf_callback_group;
      m_tf_publisher = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS(), publisher_options);
      m_tf_timer = this->create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&Wand::broadcast_tf, this),
        m_tf_callback_group);

     m_webots_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
     m_webots_step_timer = this->create_wall_timer(
       std::chrono::microseconds(static_cast<long long>(m_wb_driver->get_time_step() * 1000.0)),
       std::bind(&Wand::webots_step_timer_callback, this),
       m_webots_callback_group);
    }

  void webots_step_timer_callback()
  {
    if (m_wb_driver) {
      if (!m_wb_driver->step()) {
        m_wb_driver.reset();
        this->shutdown();
      }
    }
  }

  ~Wand()
  {
    m_tf_publisher.reset();
    m_webots_step_timer->cancel();
    m_webots_step_timer.reset();
    m_wb_driver.reset();
  }

  void broadcast_tf()
  { 
    if (!m_wb_driver || !m_tf_publisher) return;  
    Eigen::Affine3d pose = m_wb_driver->get_robot_pose();    
    
    geometry_msgs::msg::TransformStamped transformStamped; 
    transformStamped.header.stamp = this->now();  
    transformStamped.header.frame_id = "world";  
    transformStamped.child_frame_id = m_wb_driver->get_robot_name(); 
    transformStamped.transform.translation.x = pose.translation().x();  
    transformStamped.transform.translation.y = pose.translation().y(); 
    transformStamped.transform.translation.z = pose.translation().z();  
    
    Eigen::Quaterniond q(pose.linear()); 
    transformStamped.transform.rotation.x = q.x();  
    transformStamped.transform.rotation.y = q.y(); 
    transformStamped.transform.rotation.z = q.z();  
    transformStamped.transform.rotation.w = q.w();
    
    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms.push_back(transformStamped);
    m_tf_publisher->publish(tf_message);
  }

  /**
   * Lifecycle callbacks.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state)
  {
    (void)state;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_activate(state);
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state)
  {
    LifecycleNode::on_deactivate(state);
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state)
  {
    (void)state;
    RCLCPP_INFO(get_logger(), "Shutting down cleanly.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  private: 
    uint8_t m_id; 

    int m_webots_port;
    bool m_webots_use_tcp; 
    std::string m_webots_tcp_ip; 

    std::shared_ptr<WebotsWandDriver> m_wb_driver;

  private: 


    std::shared_ptr<rclcpp::CallbackGroup> m_webots_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_webots_step_timer;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> m_tf_publisher;
    std::shared_ptr<rclcpp::CallbackGroup> m_tf_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_tf_timer;

};



int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<Wand> node;
  try {
    node = std::make_shared<Wand>(options);
  } catch (const WebotsInitException &e) {
    std::cerr << "Failed to initialize Wand node: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  while (rclcpp::ok() &&  !(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)) executor.spin_some();
  executor.remove_node(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

//export LD_LIBRARY_PATH=/usr/local/webots/lib/controller:$LD_LIBRARY_PATH
//ros2 run --prefix "$WEBOTS_HOME/webots-controller --robot-name=cf1" webots_cpp test_driver 