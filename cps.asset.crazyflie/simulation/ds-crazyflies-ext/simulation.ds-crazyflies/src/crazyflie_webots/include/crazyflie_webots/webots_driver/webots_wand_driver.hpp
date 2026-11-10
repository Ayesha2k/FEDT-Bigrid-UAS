#pragma once
#include "crazyflie_webots/webots_driver/webots_robot_driver.hpp"

#define WB_ALLOW_MIXING_C_AND_CPP_API

#include <webots/keyboard.h>

class WebotsWandDriver : public WebotsRobotDriver
{
public:
    WebotsWandDriver(int id, const int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip);

    bool step() override;
private: 
    int m_id;

    bool m_selected = false;

    double m_yaw_desired = 0.0;
    bool m_flip_desired = false;
};