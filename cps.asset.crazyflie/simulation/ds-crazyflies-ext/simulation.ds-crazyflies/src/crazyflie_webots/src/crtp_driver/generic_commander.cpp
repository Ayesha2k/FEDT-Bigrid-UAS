#include "crazyflie_webots/crtp_driver/generic_commander.hpp"

GenericCommander::GenericCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> services,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging,
    std::shared_ptr<WebotsCrazyflieDriver> driver) {
  callbacks_=base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options; options.callback_group=callbacks_;
  using namespace crazyflie_interfaces::msg;
  const auto logger=logging->get_logger();
  subscriptions_.push_back(rclcpp::create_subscription<Position>(topics,"~/cmd_position",10,
    [driver,logger](Position::SharedPtr p) {
      try { double stamp=p->header.stamp.sec+p->header.stamp.nanosec*1e-9;
        driver->motion.position({p->x,p->y,p->z},p->yaw*M_PI/180,stamp>0 ? stamp : -1); }
      catch(const std::exception& e) { RCLCPP_ERROR(logger,"Position rejected: %s",e.what()); }
    },options));
  subscriptions_.push_back(rclcpp::create_subscription<FullState>(topics,"~/cmd_full_state",10,
    [driver,logger](FullState::SharedPtr p) {
      try {
        ds_motion::Reference r; const auto& v=p->twist.linear; const auto& w=p->twist.angular;
        const auto& q=p->pose.orientation; const auto& x=p->pose.position;
        r.position={x.x,x.y,x.z}; r.orientation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);
        r.velocity={v.x,v.y,v.z}; r.acceleration={p->acc.x,p->acc.y,p->acc.z}; r.omega={w.x,w.y,w.z};
        double stamp=p->header.stamp.sec+p->header.stamp.nanosec*1e-9;
        driver->motion.full_state(r,stamp>0 ? stamp : -1);
      } catch(const std::exception& e) { RCLCPP_ERROR(logger,"FullState rejected: %s",e.what()); }
    },options));
  subscriptions_.push_back(rclcpp::create_subscription<Hover>(topics,"~/cmd_hover",10,
    [driver,logger](Hover::SharedPtr p) {
      try { double stamp=p->header.stamp.sec+p->header.stamp.nanosec*1e-9;
        driver->motion.hover(p->vx,p->vy,p->yaw_rate*M_PI/180,p->z_distance,stamp>0 ? stamp : -1); }
      catch(const std::exception& e) { RCLCPP_ERROR(logger,"Hover rejected: %s",e.what()); }
    },options));
  subscriptions_.push_back(rclcpp::create_subscription<VelocityWorld>(topics,"~/cmd_velocity_world",10,
    [driver,logger](VelocityWorld::SharedPtr p) {
      try { ds_motion::Reference r; auto pose=driver->motion.pose();
        r.position=pose.position; r.orientation=pose.orientation;
        r.velocity={p->vel.x,p->vel.y,p->vel.z}; r.omega.z()=p->yaw_rate*M_PI/180;
        driver->motion.full_state(r);
      } catch(const std::exception& e) { RCLCPP_ERROR(logger,"Velocity rejected: %s",e.what()); }
    },options));
  using Release=crazyflie_interfaces::srv::NotifySetpointsStop;
  release_=rclcpp::create_service<Release>(base,services,"~/notify_setpoints_stop",
    [driver](Release::Request::SharedPtr p,Release::Response::SharedPtr) {
      if(driver->motion.matches(p->group_mask)) driver->motion.release(p->remain_valid_millisecs);
    },rmw_qos_profile_services_default,callbacks_);
}
