#include <cstdio>
#include <mutex>
#include <map>
#include <sys/wait.h>    
#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/srv/add_crazyflie.hpp"
#include "crazyflie_interfaces/srv/remove_crazyflie.hpp"
#include "crazyflie_interfaces/msg/pose_named_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "crazyflie_webots_gateway/crazyflie_lifecycle_client.hpp"


#include "signal.h"

class GatewayException : public std::runtime_error
{
public:
  explicit GatewayException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

std::atomic_bool sigint_received(false);
std::atomic_bool gateway_shutdown_done(false);

class Gateway : public rclcpp::Node
{
public:
    Gateway()
    : Node("crazyflie_webots_gateway")
    , p_webots_port(declare_parameter("webots_port", rclcpp::ParameterValue(1234), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , p_webots_use_tcp(declare_parameter("webots_use_tcp", rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<bool>())
    , p_webots_tcp_ip(declare_parameter("webots_tcp_ip", rclcpp::ParameterValue("127.0.0.1"), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>()) 
    {
      m_clock_publisher=this->create_publisher<rosgraph_msgs::msg::Clock>("/clock",10);
      m_clock_subscription=this->create_subscription<crazyflie_interfaces::msg::PoseNamedArray>("/cf_positions",10,
        [this](crazyflie_interfaces::msg::PoseNamedArray::SharedPtr poses) {
          std::lock_guard<std::mutex> lock(m_clock_mutex);
          auto stamp=rclcpp::Time(poses->header.stamp).nanoseconds();
          if(stamp<m_clock_ns && stamp+2000000000LL>=m_clock_ns) return;
          m_clock_ns=stamp;
          rosgraph_msgs::msg::Clock clock; clock.clock=poses->header.stamp;
          m_clock_publisher->publish(clock);
        });
      m_lifecycle_client_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


      auto service_qos = rmw_qos_profile_services_default;
      service_qos.depth = 100; // This way it is possible to queue up multiple add requestst
      m_gateway_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      m_add_crazyflie_service = this->create_service<crazyflie_interfaces::srv::AddCrazyflie>(
        "~/add_crazyflie",
        std::bind(&Gateway::add_crazyflie_callback, this, std::placeholders::_1, std::placeholders::_2),
        service_qos,
        m_gateway_callback_group);
      
      m_remove_crazyflie_service = this->create_service<crazyflie_interfaces::srv::RemoveCrazyflie>(
        "~/remove_crazyflie",
        std::bind(&Gateway::remove_crazyflie_callback, this, std::placeholders::_1, std::placeholders::_2),
        service_qos,
        m_gateway_callback_group);
    
    
      m_check_crazyflie_processes_timer = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&Gateway::check_crazyflie_processes, this),
        m_gateway_callback_group);

      RCLCPP_INFO(this->get_logger(), "CrazyflieWebotsGateway node initialized.");
    }

    ~Gateway(){}

    void check_crazyflie_processes()
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      std::vector<int> to_remove;
      for (auto &pair : m_crazyflie_processes)
      {
          int id = pair.first;
          pid_t pid = std::get<0>(pair.second);
          int status;
          pid_t result = waitpid(pid, &status, WNOHANG);
          if (result == 0) continue; // still running
          else if (result == -1) to_remove.push_back(id); // There was an error checking the process
          else to_remove.push_back(id);
      }

      for (int id : to_remove)
      {
          RCLCPP_INFO(this->get_logger(), "Crazyflie process %d has exited. Removing.", id);
          m_crazyflie_processes.erase(id);
      }

      static bool shutdown_completed = false;
      if (sigint_received.load() && !shutdown_completed)
      {
         shutdown_completed = true;
         if (m_crazyflie_processes.empty()) gateway_shutdown_done.store(true);
         else RCLCPP_INFO(this->get_logger(), "Shutting down all crazyflies due to SIGINT.");
         for (auto &pair : m_crazyflie_processes)
         {
            RCLCPP_INFO(this->get_logger(), "Shutting down crazyflie with id %d.", pair.first);
            auto cf_client = std::get<1>(pair.second);
            cf_client->shutdown_crazyflie_async();
         }
      }
    }

    void add_crazyflie_callback(
      const std::shared_ptr<crazyflie_interfaces::srv::AddCrazyflie::Request> request,
      std::shared_ptr<crazyflie_interfaces::srv::AddCrazyflie::Response> response)
    {
      int id;
      try {
        id = uri_to_id(request->uri);
      } catch (const GatewayException & e) {
          response->success = false;
          response->msg = e.what();
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Adding crazyflie with id: %d.", id);

      {
        std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);
        if (m_crazyflie_processes.find(id) != m_crazyflie_processes.end())
        {
            RCLCPP_WARN(this->get_logger(), "Crazyflie with id %d already exists.", id);
            response->success = true; // Idempotent compatibility add.
            return;
        }

        pid_t pid = fork();
        if (pid == 0) 
        {
            setpgid(0,0); //Create new process group so that we can kill all child processes later       
            std::string node_arg = "__node:=cf" + std::to_string(id);  
            std::string id_arg = "id:=" + std::to_string(id);
            std::string webots_port_arg = "webots_port:=" + std::to_string(p_webots_port);
            std::string webots_use_tcp_arg = p_webots_use_tcp ? "webots_use_tcp:=true" : "webots_use_tcp:=false";
            std::string webots_tcp_ip_arg = "webots_tcp_ip:=" + p_webots_tcp_ip;
            
            std::vector<char*> argv;
            argv.push_back(const_cast<char*>("ros2"));
            argv.push_back(const_cast<char*>("run"));
            argv.push_back(const_cast<char*>("crazyflie_webots"));
            argv.push_back(const_cast<char*>("crazyflie"));
            argv.push_back(const_cast<char*>("--ros-args"));
            argv.push_back(const_cast<char*>("-p"));
            argv.push_back(const_cast<char*>(id_arg.c_str()));
            argv.push_back(const_cast<char*>("-p"));
            argv.push_back(const_cast<char*>(webots_port_arg.c_str())); 
            argv.push_back(const_cast<char*>("-p"));
            argv.push_back(const_cast<char*>(webots_use_tcp_arg.c_str()));
            argv.push_back(const_cast<char*>("-p"));
            argv.push_back(const_cast<char*>(webots_tcp_ip_arg.c_str()));
            argv.push_back(const_cast<char*>("-r"));
            argv.push_back(const_cast<char*>(node_arg.c_str()));  // safe: node_arg lives
            argv.push_back(nullptr);
    
            execvp("ros2", argv.data());
            _exit(1);
        }

        RCLCPP_DEBUG(this->get_logger(), "Started crazyflie process with PID %d for id %d.", pid, id);

        std::shared_ptr<CrazyflieLifecycleClient> cf_lifecycle_client = 
          std::make_shared<CrazyflieLifecycleClient>(
            this->get_node_topics_interface(),
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_services_interface(),
            m_lifecycle_client_callback_group,
            id,
            std::bind(&Gateway::on_crazyflie_shutdown, this, std::placeholders::_1));

        m_crazyflie_processes[id] = std::make_tuple(pid, cf_lifecycle_client);
      }

      response->success = true;
    }

    void remove_crazyflie_callback(
      const std::shared_ptr<crazyflie_interfaces::srv::RemoveCrazyflie::Request> request,
      std::shared_ptr<crazyflie_interfaces::srv::RemoveCrazyflie::Response> response)
    {
      std::unique_lock<std::mutex> lock(m_crazyflie_processes_mutex);

      int id;
      try {
        id = uri_to_id(request->uri);
      } catch (const GatewayException & e) {
          response->success = false;
          response->msg = e.what();
          return;
      }

      RCLCPP_INFO(this->get_logger(), "Remove crazyflie service called for id: %d.", id);

      response->success = true;

      auto it = m_crazyflie_processes.find(id);
      if (it != m_crazyflie_processes.end())
      {
          std::get<1>(it->second)->shutdown_crazyflie_async();
          lock.unlock();

          bool removal_success = false;
          std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
          while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(300))
          {
              lock.lock();
              removal_success = m_crazyflie_processes.find(id) == m_crazyflie_processes.end();
              lock.unlock();
              if (removal_success) break;
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }

          lock.lock();
          if (!(m_crazyflie_processes.find(id) == m_crazyflie_processes.end()))
          {
              RCLCPP_WARN(this->get_logger(), "Shutdown of crazyflie %d timed out.", id);
              pid_t pid = std::get<0>(it->second);
              kill(-pid, SIGTERM); //Kill the whole process group
              m_crazyflie_processes.erase(it);
          }
          RCLCPP_INFO(this->get_logger(), "Crazyflie with id %d removed.", id);
      } else {
          response->success = false;
          RCLCPP_WARN(this->get_logger(), "Crazyflie with id %d does not exist.", id);
      }

    }

    void on_crazyflie_shutdown(int id)
    {
      std::lock_guard<std::mutex> lock(m_crazyflie_processes_mutex);

      auto it = m_crazyflie_processes.find(id);
      if (it != m_crazyflie_processes.end())
      {
          RCLCPP_INFO(this->get_logger(), "Detected shutdown on crazyflie with id %d.", id);
          m_crazyflie_processes.erase(it);
      }

      if (m_crazyflie_processes.empty() && sigint_received.load())
      {
          RCLCPP_INFO(this->get_logger(), "All crazyflie processes have shut down. Proceeding with gateway shutdown.");
          gateway_shutdown_done.store(true);
      }      
    }

private: 
    std::mutex m_clock_mutex;
    int64_t m_clock_ns=0;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clock_publisher;
    rclcpp::Subscription<crazyflie_interfaces::msg::PoseNamedArray>::SharedPtr m_clock_subscription;
    int 
    uri_to_id(const std::string & uri)
    {
      const std::string prefix = "webots://";
      if (uri.rfind(prefix, 0) != 0) {
          throw GatewayException("URI must start with webots://");
      }

      try {
          return std::stoi(uri.substr(prefix.length()));
      } catch (...) {
          throw GatewayException("ID must be an integer");
      }
    }


private: 
    std::shared_ptr<rclcpp::CallbackGroup> m_lifecycle_client_callback_group;

    std::shared_ptr<rclcpp::CallbackGroup> m_shutdown_detector_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_shutdown_detector_timer;

    std::shared_ptr<rclcpp::CallbackGroup> m_gateway_callback_group;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::AddCrazyflie>> m_add_crazyflie_service;
    std::shared_ptr<rclcpp::Service<crazyflie_interfaces::srv::RemoveCrazyflie>> m_remove_crazyflie_service; 
    
    std::shared_ptr<rclcpp::TimerBase> m_check_crazyflie_processes_timer;

    std::mutex m_crazyflie_processes_mutex;
    std::map<int, std::tuple<pid_t, std::shared_ptr<CrazyflieLifecycleClient>>> m_crazyflie_processes;

private: 
    int p_webots_port;
    bool p_webots_use_tcp;
    std::string p_webots_tcp_ip;
};



void sigint_handler(int signum)
{
    (void)signum;
    sigint_received.store(true);
    int safey_counter = 0;
    while (!gateway_shutdown_done.load())
    { 
        safey_counter++;
        if (safey_counter > 500) break;// 3 seconds timeout
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (gateway_shutdown_done.load()) std::cerr << "Gateway shut down cleanly after SIGINT." << std::endl;
    else std::cerr << "Gateway shutdown after SIGINT timed out." << std::endl;
}


int main(int argc, char ** argv)
{
  signal(SIGINT, sigint_handler);
  // Install before rclcpp this way rclcpp will store it as a "old" handler and execute it before its own shutdown

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

  auto gateway = std::make_shared<Gateway>();
  
  executor.add_node(gateway);
  executor.spin();
  executor.remove_node(gateway->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
