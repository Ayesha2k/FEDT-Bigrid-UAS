#include "crazyflie_webots/crtp_driver/localization.hpp"
#include <sstream>
Localization::Localization(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface>,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers,
    std::shared_ptr<WebotsCrazyflieDriver> driver) {
  callbacks_=base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pose_=rclcpp::create_publisher<crazyflie_interfaces::msg::PoseNamedArray>(topics,"/cf_positions",10);
  odom_=rclcpp::create_publisher<nav_msgs::msg::Odometry>(topics,"~/odom",10);
  scan_=rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(topics,"~/scan",10);
  status_=rclcpp::create_publisher<crazyflie_interfaces::msg::Status>(topics,"~/status",10);
  diagnostic_=rclcpp::create_publisher<std_msgs::msg::String>(topics,"~/motion_status",10);
  // Exactly one clock publisher for the preserved 12-drone world.
  const auto name=driver->get_robot_name();
  // The gateway owns /clock so removing a particular drone cannot freeze peers.
  timer_=rclcpp::create_wall_timer(std::chrono::milliseconds(50),[this,driver,name]() {
    const auto pose=driver->get_robot_pose();
    const double time=driver->simulation_time();
    const builtin_interfaces::msg::Time stamp=rclcpp::Time(static_cast<int64_t>(time*1e9));
    const auto q=Eigen::Quaterniond(pose.rotation());
    nav_msgs::msg::Odometry odom;
    odom.header.stamp=stamp; odom.header.frame_id="world"; odom.child_frame_id=name;
    odom.pose.pose.position.x=pose.translation().x(); odom.pose.pose.position.y=pose.translation().y(); odom.pose.pose.position.z=pose.translation().z();
    odom.pose.pose.orientation.x=q.x(); odom.pose.pose.orientation.y=q.y(); odom.pose.pose.orientation.z=q.z(); odom.pose.pose.orientation.w=q.w();
    const auto velocity=q.conjugate()*driver->measured_velocity();
    odom.twist.twist.linear.x=velocity.x(); odom.twist.twist.linear.y=velocity.y(); odom.twist.twist.linear.z=velocity.z();
    odom_->publish(odom);
    crazyflie_interfaces::msg::PoseNamedArray array;
    array.header=odom.header;
    crazyflie_interfaces::msg::PoseNamed p; p.name=name; p.pose=odom.pose.pose;
    array.poses.push_back(p); pose_->publish(array);
    sensor_msgs::msg::LaserScan scan; scan.header.stamp=stamp; scan.header.frame_id=name;
    scan.angle_min=-M_PI; scan.angle_max=M_PI/2; scan.angle_increment=M_PI/2;
    scan.range_min=0.01; scan.range_max=2; scan.scan_time=0.05;
    const auto ranges=driver->ranges();
    for(int i=0;i<4;++i) scan.ranges.push_back(ranges[i]);
    scan_->publish(scan);
    crazyflie_interfaces::msg::Status status; status.header=odom.header;
    status.battery_voltage=4.2; status.pm_state=0;
    status.supervisor_info=status.SUPERVISOR_INFO_CAN_BE_ARMED | status.SUPERVISOR_INFO_CAN_FLY | status.SUPERVISOR_INFO_AUTO_ARM;
    if(pose.translation().z()>0.05) status.supervisor_info |= status.SUPERVISOR_INFO_IS_FLYING | status.SUPERVISOR_INFO_IS_ARMED;
    status_->publish(status);
    std_msgs::msg::String diagnostic;
    const auto target=driver->motion.target();
    diagnostic.data="{\"mode\":"+std::to_string(static_cast<int>(driver->motion.mode()))+",\"sim_time\":"+std::to_string(time)+",\"start_time\":"+std::to_string(driver->motion.start_epoch())+",\"actual_start\":"+std::to_string(driver->motion.actual_start())+",\"setpoint\":["+std::to_string(target.x())+","+std::to_string(target.y())+","+std::to_string(target.z())+"],\"reason\":\""+driver->motion.reason()+"\"}";
    diagnostic_->publish(diagnostic);
  },callbacks_,base.get(),timers.get());
}
