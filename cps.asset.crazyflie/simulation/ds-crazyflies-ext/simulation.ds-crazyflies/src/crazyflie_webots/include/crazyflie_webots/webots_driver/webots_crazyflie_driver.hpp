#pragma once
#include <string>
#include <atomic>
#include "crazyflie_webots/motion.hpp"
#include <math.h>
#include <vector>

#include "crazyflie_webots/webots_driver/webots_robot_driver.hpp"

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/robot.h>
#include <webots/device.h>
#include <webots/supervisor.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <iostream>


class WebotsCrazyflieDriver : public WebotsRobotDriver
{
public:
    WebotsCrazyflieDriver(int id, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip);

    bool step() override;
    ds_motion::Engine motion;
    double simulation_time() const { return m_sim_time.load(); }
    std::array<double, 6> ranges() const { std::lock_guard<std::mutex> lock(m_sensor_mutex); return m_ranges; }
    Eigen::Vector3d measured_velocity() const { std::lock_guard<std::mutex> lock(m_sensor_mutex); return m_velocity; }

    void set_target(Eigen::Vector3d target, double yaw_deg);

    double get_range_front();
    double get_range_back();
    double get_range_up();
    double get_range_left();
    double get_range_right();
    double get_range_zrange();


    double get_battery_voltage();
    double get_charge_current();
    double get_charge_state();
    bool can_fly();
    bool is_flying();
    bool is_tumbled();


private:
    int m_id;
    std::atomic<double> m_sim_time{0};
    mutable std::mutex m_sensor_mutex;
    std::array<double, 6> m_ranges{};
    Eigen::Vector3d m_velocity=Eigen::Vector3d::Zero();
    Eigen::Vector3d m_previous_position=Eigen::Vector3d::Zero();

private: 
    std::vector<double> m_target = {0.0, 0.0, 0.0};
    double m_target_yaw_deg = 0.0;

private: 
    WbDeviceTag m_m1_motor;
    WbDeviceTag m_m2_motor; 
    WbDeviceTag m_m3_motor; 
    WbDeviceTag m_m4_motor; 

    WbDeviceTag m_gps;

    WbDeviceTag m_ranger_front;
    WbDeviceTag m_ranger_back;
    //WbDeviceTag m_ranger_up;
    WbDeviceTag m_ranger_left;
    WbDeviceTag m_ranger_right;
    WbDeviceTag m_zranger; 

    

    // Initialize error accumulators
    double m_forward_integral = 0.0;
    double m_sideways_integral = 0.0;
    double m_up_integral = 0.0;

    double m_past_time;

private: 
    enum class ChargeState {
        BATTERY, 
        CHARGING, 
        CHARGED, 
        LOWPOWER, 
        SHUTDOWN
    };

};