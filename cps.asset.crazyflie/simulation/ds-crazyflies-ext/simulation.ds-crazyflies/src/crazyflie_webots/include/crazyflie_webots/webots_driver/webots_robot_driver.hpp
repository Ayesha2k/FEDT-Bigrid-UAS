#pragma once
#include <string>
#include <mutex>

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/robot.h>
#include <webots/supervisor.h>


#include <Eigen/Dense>

#include <stdexcept>

class WebotsInitException : public std::runtime_error {
public:
    explicit WebotsInitException(const std::string &msg)
        : std::runtime_error(msg) {}
};

class WebotsRobotDriver
{
public:
    WebotsRobotDriver(const std::string robot_name, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip);   
    virtual ~WebotsRobotDriver();

    virtual bool step();
    double get_time_step();

    std::string get_robot_name() const { return m_robot_name; }

    Eigen::Affine3d get_robot_pose();

    bool is_connected() const { return m_connected; }
private:
    Eigen::Affine3d read_robot_pose();
    mutable std::mutex m_pose_mutex;
    Eigen::Affine3d m_cached_pose = Eigen::Affine3d::Identity();
    std::string m_robot_name;
    int m_webots_port;
    bool m_webots_use_tcp;
    std::string m_webots_tcp_ip;

private: 
    
    double m_time_step;
    bool m_connected;

protected:
    WbNodeRef m_robot_node;

    WbFieldRef m_position_field;
    WbFieldRef m_rotation_field;

};    