#include "crazyflie_webots/webots_driver/webots_wand_driver.hpp"
#include <iostream>

WebotsWandDriver::WebotsWandDriver(int id, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip)
    : WebotsRobotDriver("Wand" + std::to_string(id), webots_port, webots_use_tcp, webots_tcp_ip)
    , m_id(id)
{
    std::cout << "====== Controls =======" << std::endl
    << " The Wand can be controlled from your keyboard!" << std::endl
    << " All controllable movement is in world coordinates" << std::endl
    << "- Use the up, back, right and left button to move in the horizontal plane" << std::endl
    << "- Use Q and E to rotate around yaw" << std::endl
    << "- Use W and S to go up and down" << std::endl
    << "- Use A and D to switch between toggle states" << std::endl
    << "- Use number keys to switch between wand" << std::endl
    << "=======================" << std::endl;

    wb_keyboard_enable(static_cast<int>(get_time_step()));
}

bool 
WebotsWandDriver::step()
{
    if (!WebotsRobotDriver::step()) return false;

    double forward_speed = 0.0;
    double sideward_speed = 0.0;
    double upward_speed = 0.0;

    int key = wb_keyboard_get_key();
    while (key > 0) {
    switch (key) {
        case WB_KEYBOARD_UP:
        if (m_selected) forward_speed = 0.5;
        break;
        case WB_KEYBOARD_DOWN:
        if (m_selected) forward_speed = -0.5;
        break;
        case WB_KEYBOARD_RIGHT:
        if (m_selected) sideward_speed = -0.5;
        break;
        case WB_KEYBOARD_LEFT:
        if (m_selected) sideward_speed = +0.5;
        break;
        case 'A':
        //rotation[2] = 1;
        if (m_selected) m_flip_desired = true;
        break;
        case 'D':
        if (m_selected) m_flip_desired = false;
        break;
        case 'W':
        if (m_selected) upward_speed = 0.5;
        break;
        case 'S':
        if (m_selected) upward_speed = -0.5;
        break;
        case 'Q':
        if (m_selected) m_yaw_desired += 0.2;
        //rotation[3] += 0.2;
        break;
        case 'E':
        if (m_selected) m_yaw_desired -= 0.2;
        //rotation[3] -= 0.2;
        break;
        default:
        {
        int nbr_int = key - '0';
        if (nbr_int >= 0 && nbr_int <= 9) {
            if (nbr_int == m_id) m_selected = true;
            else m_selected = false;
            }
        }
    }
    key = wb_keyboard_get_key();
    }

    double rotation[4] = {0,0,1,0};
    rotation[0] = cos(m_yaw_desired/2) + cos(m_flip_desired/2); // to be 0 if flip = 0 to be some thing if flip = 3.14
    rotation[2] = sin(m_yaw_desired/2) + sin(m_flip_desired/2); // to be 1 if flip = 0 to be something if flip = 3.14
      
    if (!m_flip_desired) {
          rotation[0] = 0;
          rotation[1] = 0;
          rotation[2] = 1;
          rotation[3] = m_yaw_desired;
    } else {
          rotation[0] = cos(m_yaw_desired/2);
          rotation[1] = sin(m_yaw_desired/2);
          rotation[2] = 0;
          rotation[3] = 3.14;
    }

    const double vel[6] = {forward_speed, sideward_speed, upward_speed, 0, 0,0};
    wb_supervisor_node_set_velocity(m_robot_node, vel);
    wb_supervisor_field_set_sf_rotation(m_rotation_field, (const double *)rotation);

    return true;
}