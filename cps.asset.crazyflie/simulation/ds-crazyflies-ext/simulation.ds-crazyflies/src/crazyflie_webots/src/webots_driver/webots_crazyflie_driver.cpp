#include "crazyflie_webots/webots_driver/webots_crazyflie_driver.hpp"
#include <cstdio>
#include <iostream>

WebotsCrazyflieDriver::WebotsCrazyflieDriver(int id, int webots_port, bool webots_use_tcp, const std::string &webots_tcp_ip)
      : WebotsRobotDriver("cf" + std::to_string(id), webots_port, webots_use_tcp, webots_tcp_ip) 
      , m_id(id)
{
    // Initialize motors
    m_m1_motor = wb_robot_get_device("m1_motor");
    wb_motor_set_position(m_m1_motor, INFINITY);
    m_m2_motor = wb_robot_get_device("m2_motor");
    wb_motor_set_position(m_m2_motor, INFINITY);
    m_m3_motor = wb_robot_get_device("m3_motor");
    wb_motor_set_position(m_m3_motor, INFINITY);
    m_m4_motor = wb_robot_get_device("m4_motor");
    wb_motor_set_position(m_m4_motor, INFINITY);
  
    // Initialize sensors
    m_gps = wb_robot_get_device("gps");
    wb_gps_enable(m_gps, static_cast<int>(get_time_step()));
    m_zranger = wb_robot_get_device("zrange");
    m_ranger_front = wb_robot_get_device("range_front");
    m_ranger_back = wb_robot_get_device("range_back");
    //m_ranger_up = wb_robot_get_device("range_up");
    m_ranger_left = wb_robot_get_device("range_left");
    m_ranger_right = wb_robot_get_device("range_right");

    Eigen::Vector3d pos = get_robot_pose().translation();
    m_target = {pos.x(), pos.y(), pos.z()};
    
    for(auto tag : {m_ranger_front,m_ranger_back,m_ranger_left,m_ranger_right,m_zranger})
        wb_distance_sensor_enable(tag, static_cast<int>(get_time_step()));
    m_past_time = wb_robot_get_time();
    m_sim_time.store(m_past_time);
    m_previous_position=pos;
    ds_motion::Pose initial; initial.position=pos; initial.orientation=Eigen::Quaterniond(get_robot_pose().linear());
    motion.tick(m_past_time,initial);
    auto synchronization=wb_supervisor_node_get_field(m_robot_node,"synchronization");
    if(synchronization) wb_supervisor_field_set_sf_bool(synchronization,false);
}


namespace {
double metres(WbDeviceTag tag) {
    const double value=wb_distance_sensor_get_value(tag);
    const int n=wb_distance_sensor_get_lookup_table_size(tag);
    const double* table=wb_distance_sensor_get_lookup_table(tag);
    if(n<2 || !table || !std::isfinite(value)) return INFINITY;
    for(int i=1;i<n;++i) {
        const double a=table[(i-1)*3+1], b=table[i*3+1];
        if(value>=std::min(a,b) && value<=std::max(a,b) && a!=b) {
            double distance=table[(i-1)*3]+(value-a)/(b-a)*(table[i*3]-table[(i-1)*3]);
            return distance>=table[(n-1)*3] ? INFINITY : distance;
        }
    }
    return INFINITY;
}
}
bool
WebotsCrazyflieDriver::step()
{
    if(!WebotsRobotDriver::step()) return false;
    const double now=wb_robot_get_time();
    const double dt=now-m_past_time;
    const auto pose=get_robot_pose();
    ds_motion::Pose measured; measured.position=pose.translation(); measured.orientation=Eigen::Quaterniond(pose.linear());
    auto command=motion.tick(now,measured);
    const double vel[6]={command.linear.x(),command.linear.y(),command.linear.z(),command.angular.x(),command.angular.y(),command.angular.z()};
    wb_supervisor_node_set_velocity(m_robot_node,vel);
    {
        std::lock_guard<std::mutex> lock(m_sensor_mutex);
        if(dt>0) m_velocity=(measured.position-m_previous_position)/dt;
        m_previous_position=measured.position;
        m_ranges={metres(m_ranger_back),metres(m_ranger_right),metres(m_ranger_front),metres(m_ranger_left),metres(m_zranger),INFINITY};
    }
    m_sim_time.store(now);
    int speed=measured.position.z()>0.025 || command.linear.z()>0 ? 48 : 0;
    wb_motor_set_velocity(m_m1_motor,-speed); wb_motor_set_velocity(m_m2_motor,speed);
    wb_motor_set_velocity(m_m3_motor,-speed); wb_motor_set_velocity(m_m4_motor,speed);
    m_past_time=now;
    return true;
}

void
WebotsCrazyflieDriver::set_target(Eigen::Vector3d target, double yaw_deg)
{
    motion.position(target,yaw_deg*M_PI/180.0);

}

double
WebotsCrazyflieDriver::get_battery_voltage()
{
    return 4.2;    // Dummy value for battery voltage
}

double
WebotsCrazyflieDriver::get_charge_current()
{
    return 0.0;    // Dummy value for charge current
}

double
WebotsCrazyflieDriver::get_charge_state()
{
    return double(ChargeState::CHARGED); // Dummy value for charge state
} 

bool
WebotsCrazyflieDriver::can_fly()
{
    return true; // Dummy value for can fly
}

bool
WebotsCrazyflieDriver::is_flying()
{
    double z_global = get_robot_pose().translation().z();
    return (z_global > 0.05); // Consider flying if above 5 cm
}

bool
WebotsCrazyflieDriver::is_tumbled()
{
    return false; // Dummy value for is tumbled
}

double
WebotsCrazyflieDriver::get_range_front()
{ return ranges()[2];
}

double
WebotsCrazyflieDriver::get_range_back()
{ return ranges()[0];
}

double
WebotsCrazyflieDriver::get_range_up()
{ return ranges()[5];
}   

double
WebotsCrazyflieDriver::get_range_left()
{ return ranges()[3];
}   

double
WebotsCrazyflieDriver::get_range_right()
{ return ranges()[1];
}   

double
WebotsCrazyflieDriver::get_range_zrange()
{ return ranges()[4];
}
