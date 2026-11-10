#include "crazyflie_webots/crtp_driver/hl_commander.hpp"
#include <iomanip>
#include <sstream>

namespace {
template<class F> void run(const std::string& action,F f,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,rclcpp::Logger logger) {
  std_msgs::msg::String report;
  std::ostringstream text;
  try { f(); text << "{\"action\":" << std::quoted(action) << ",\"success\":true}"; }
  catch(const std::exception& e) {
    text << "{\"action\":" << std::quoted(action) << ",\"success\":false,\"error\":" << std::quoted(e.what()) << "}";
    RCLCPP_ERROR(logger,"%s rejected: %s",action.c_str(),e.what());
  }
  report.data=text.str(); publisher->publish(report);
}
double seconds(const builtin_interfaces::msg::Duration& d) { return d.sec+d.nanosec*1e-9; }
ds_motion::Piece piece(const crazyflie_interfaces::msg::TrajectoryPolynomialPiece& p) {
  ds_motion::Piece r; r.duration=seconds(p.duration);
  auto copy=[](const auto& from,auto& to) {
    if(from.empty() || from.size()>8) throw std::invalid_argument("Polynomial needs 1..8 coefficients");
    std::copy(from.begin(),from.end(),to.begin());
  };
  copy(p.poly_x,r.x); copy(p.poly_y,r.y); copy(p.poly_z,r.z); copy(p.poly_yaw,r.yaw);
  return r;
}
}

HighLevelCommander::HighLevelCommander(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> services,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging,
    std::shared_ptr<WebotsCrazyflieDriver> driver) {
  callbacks_=base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  report_=rclcpp::create_publisher<std_msgs::msg::String>(topics,"~/control_result",rclcpp::QoS(10));
  const auto logger=logging->get_logger();
  auto report=report_;
  using namespace crazyflie_interfaces::srv;
  services_.push_back(rclcpp::create_service<Takeoff>(base,services,"~/takeoff",
    [driver,report,logger](Takeoff::Request::SharedPtr r,Takeoff::Response::SharedPtr) {
      run("takeoff",[&] { if(driver->motion.matches(r->group_mask)) {
        auto p=driver->motion.pose().position; p.z()=r->height;
        driver->motion.timed(p,r->yaw,seconds(r->duration));
      }},report,logger);
    },rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<Land>(base,services,"~/land",
    [driver,report,logger](Land::Request::SharedPtr r,Land::Response::SharedPtr) {
      run("land",[&] { if(driver->motion.matches(r->group_mask)) {
        auto p=driver->motion.pose().position; p.z()=r->height+0.02;
        driver->motion.timed(p,r->yaw,seconds(r->duration),true);
      }},report,logger);
    },rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<GoTo>(base,services,"~/go_to",
    [driver,report,logger](GoTo::Request::SharedPtr r,GoTo::Response::SharedPtr) {
      run("go_to",[&] { if(driver->motion.matches(r->group_mask)) {
        ds_motion::Vec p(r->goal.x,r->goal.y,r->goal.z); double yaw=r->yaw;
        if(r->relative) { auto current=driver->motion.pose(); p+=current.position;
          const auto rotation=current.orientation.toRotationMatrix(); yaw+=std::atan2(rotation(1,0),rotation(0,0)); }
        driver->motion.timed(p,yaw,seconds(r->duration));
      }},report,logger);
    },rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<UploadTrajectory>(base,services,"~/upload_trajectory",
    [driver,report,logger](UploadTrajectory::Request::SharedPtr r,UploadTrajectory::Response::SharedPtr) {
      run("upload_trajectory",[&] {
        std::vector<ds_motion::Piece> pieces; for(const auto& p:r->pieces) pieces.push_back(piece(p));
        driver->motion.upload(r->trajectory_id,r->piece_offset,pieces);
      },report,logger);
    },rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<StartTrajectory>(base,services,"~/start_trajectory",
    [driver,report,logger](StartTrajectory::Request::SharedPtr r,StartTrajectory::Response::SharedPtr) {
      run("start_trajectory",[&] { if(driver->motion.matches(r->group_mask))
        driver->motion.start(r->trajectory_id,r->timescale,r->reversed,r->relative,driver->motion.time());
      },report,logger);
    },rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<SetGroupMask>(base,services,"~/set_group_mask",
    [driver](SetGroupMask::Request::SharedPtr r,SetGroupMask::Response::SharedPtr) { driver->motion.set_group(r->group_mask); },
    rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<Stop>(base,services,"~/stop",
    [driver](Stop::Request::SharedPtr r,Stop::Response::SharedPtr) { if(driver->motion.matches(r->group_mask)) driver->motion.hold(); },
    rmw_qos_profile_services_default,callbacks_));
  services_.push_back(rclcpp::create_service<std_srvs::srv::Trigger>(base,services,"~/trajectory_status",
    [driver](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr r) {
      r->success=true; std::ostringstream s; s << "{\"ids\":["; bool first=true;
      for(auto id:driver->motion.ids()) { if(!first) s << ","; s << id; first=false; }
      s << "],\"group\":" << driver->motion.group() << "}"; r->message=s.str();
    },rmw_qos_profile_services_default,callbacks_));
  rclcpp::SubscriptionOptions options; options.callback_group=callbacks_;
  // Private schedule: [id, scale, reverse, relative, mask, shared_simulation_epoch].
  // The public StartTrajectory service type remains unchanged.
  schedule_=rclcpp::create_subscription<std_msgs::msg::Float64MultiArray>(topics,"/_ds/trajectory_schedule",10,
    [driver,report,logger](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      run("scheduled_trajectory",[&] {
        const auto& d=msg->data;
        if(d.size()!=6) throw std::invalid_argument("Invalid schedule");
        for(auto x:d) if(!std::isfinite(x)) throw std::invalid_argument("Invalid schedule");
        if(driver->motion.matches(static_cast<unsigned>(d[4]))) {
          if(d[5]<driver->motion.time()) throw std::invalid_argument("Shared start epoch already passed");
          driver->motion.start(static_cast<int>(d[0]),d[1],d[2]!=0,d[3]!=0,d[5]);
        }
      },report,logger);
    },options);
}
