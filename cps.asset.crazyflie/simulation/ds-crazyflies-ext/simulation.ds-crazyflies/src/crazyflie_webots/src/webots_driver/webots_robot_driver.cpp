#include "crazyflie_webots/webots_driver/webots_robot_driver.hpp"
#include <iostream>
#include <atomic>
#include <memory>
#include <thread>      // for std::thread and std::this_thread::sleep_for
#include <chrono>      // for std::chrono::steady_clock and durations
#include <signal.h>
WebotsRobotDriver::WebotsRobotDriver(const std::string robot_name, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip)
      : m_robot_name(robot_name)
      , m_webots_port(webots_port)
      , m_webots_use_tcp(webots_use_tcp)
      , m_webots_tcp_ip(webots_tcp_ip)
{
    std::string url;
    if (m_webots_use_tcp) {
        url = "tcp://" + m_webots_tcp_ip + ":" + std::to_string(m_webots_port) + "/" + robot_name;
    } else {
        url = "ipc://" + std::to_string(m_webots_port) + "/" + robot_name;
    }
    setenv("WEBOTS_CONTROLLER_URL", url.c_str(), 1);

    // std::cerr << "Connecting to Webots robot with URL: " << url << std::endl;

    auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(15);

    auto init_success = std::make_shared<std::atomic_bool>(false);
    std::thread initThread([init_success](){
        // The signal handler would be overridden by Webots, so we need to save and restore it
        struct sigaction old_sigint;
        sigaction(SIGINT, NULL, &old_sigint); // get current handler
        wb_robot_init();
        sigaction(SIGINT, &old_sigint, NULL); // Restore ROS SIGINT handler

        init_success->store(true);
    });
    while (std::chrono::steady_clock::now() - start < timeout) {
        if (init_success->load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!init_success->load()) {
        m_connected = false;
        initThread.detach(); // let the thread clean up itself
        throw WebotsInitException("Webots robot initialization timeout after 15 seconds. Is Webots running with the correct robot?");
    } else {
        initThread.join();
        // continue normally
    }


    m_connected = true;
    m_time_step = wb_robot_get_basic_time_step();

    m_robot_node = wb_supervisor_node_get_self();
    m_position_field = wb_supervisor_node_get_field(m_robot_node, "translation");
    m_rotation_field = wb_supervisor_node_get_field(m_robot_node, "rotation");

    m_cached_pose = read_robot_pose();
    // Cache all Webots state in the stepping thread.    
}

WebotsRobotDriver::~WebotsRobotDriver()
{
    if (m_connected) 
    {
        wb_robot_cleanup();
    }
}

bool
WebotsRobotDriver::step()
{
    if (m_connected && wb_robot_step(m_time_step) == -1)
    {
        m_connected = false;
        return false;
    }
    { std::lock_guard<std::mutex> lock(m_pose_mutex); m_cached_pose=read_robot_pose(); }
    return true;
}

double
WebotsRobotDriver::get_time_step()
{
    return m_time_step;
}

Eigen::Affine3d
WebotsRobotDriver::read_robot_pose()
{
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    const double *position = wb_supervisor_field_get_sf_vec3f(m_position_field);
    const double *rotation = wb_supervisor_field_get_sf_rotation(m_rotation_field);
    pose.translation() << position[0], position[1], position[2];
    Eigen::AngleAxisd angle_axis(rotation[3], Eigen::Vector3d(rotation[0], rotation[1], rotation[2]));
    pose.linear() = angle_axis.toRotationMatrix();
    return pose;
}
Eigen::Affine3d WebotsRobotDriver::get_robot_pose() {
    std::lock_guard<std::mutex> lock(m_pose_mutex);
    return m_cached_pose;
}
