#pragma once
// Simulation-only motion engine. No Webots, ROS, radio or firmware dependencies.
#include <Eigen/Geometry>
#include <array>
#include <vector>
#include <map>
#include <mutex>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <string>

namespace ds_motion {
using Vec=Eigen::Vector3d;
struct Pose { Vec position=Vec::Zero(); Eigen::Quaterniond orientation=Eigen::Quaterniond::Identity(); };
struct Reference : Pose { Vec velocity=Vec::Zero(), acceleration=Vec::Zero(), omega=Vec::Zero(); };
struct Output { Vec linear=Vec::Zero(), angular=Vec::Zero(); };
struct Piece {
  double duration=0;
  std::array<double,8> x{},y{},z{},yaw{};
};
enum class Mode { Hold, Position, Timed, FullState, Hover, Trajectory };
inline double poly(const std::array<double,8>& c,double t,int order) {
  double sum=0;
  for(int i=7;i>=order;--i) {
    double factor=1;
    for(int k=0;k<order;++k) factor*=i-k;
    sum=sum*t+c[i]*factor;
  }
  return sum;
}
inline double duration(const std::vector<Piece>& p) {
  double d=0;
  for(const auto& s:p) d+=s.duration;
  return d;
}
inline Reference evaluate(const std::vector<Piece>& pieces,double elapsed,double scale,bool reverse) {
  if(pieces.empty() || !std::isfinite(scale) || scale<=0) throw std::invalid_argument("Invalid trajectory");
  const double total=duration(pieces);
  double t=std::clamp(elapsed/scale,0.0,total);
  if(reverse) t=total-t;
  const Piece* piece=&pieces.back();
  // A rounded duration sum can exceed the final duration by a few ulps.
  // Clamp within the final piece instead of subtracting it and wrapping to zero.
  for(const auto& p:pieces) {
    piece=&p;
    if(t<=p.duration || piece==&pieces.back()) break;
    t-=p.duration;
  }
  t=std::clamp(t,0.0,piece->duration);
  const double s=(reverse ? -1.0 : 1.0)/scale;
  Reference r;
  r.position=Vec(poly(piece->x,t,0),poly(piece->y,t,0),poly(piece->z,t,0));
  r.velocity=s*Vec(poly(piece->x,t,1),poly(piece->y,t,1),poly(piece->z,t,1));
  r.acceleration=s*s*Vec(poly(piece->x,t,2),poly(piece->y,t,2),poly(piece->z,t,2));
  r.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(poly(piece->yaw,t,0),Vec::UnitZ()));
  r.omega.z()=s*poly(piece->yaw,t,1);
  return r;
}

class Engine {
 public:
  bool matches(unsigned mask) const { std::lock_guard<std::mutex> l(lock_); return mask==0 || (mask&group_)!=0; }
  void set_group(unsigned mask) { std::lock_guard<std::mutex> l(lock_); group_=mask; }
  unsigned group() const { std::lock_guard<std::mutex> l(lock_); return group_; }
  Mode mode() const { std::lock_guard<std::mutex> l(lock_); return mode_; }
  std::string reason() const { std::lock_guard<std::mutex> l(lock_); return reason_; }
  Pose pose() const { std::lock_guard<std::mutex> l(lock_); return measured_; }
  Vec target() const { std::lock_guard<std::mutex> l(lock_); return setpoint_.position; }
  double time() const { std::lock_guard<std::mutex> l(lock_); return now_; }
  double start_epoch() const { std::lock_guard<std::mutex> l(lock_); return start_; }
  double actual_start() const { std::lock_guard<std::mutex> l(lock_); return actual_start_; }
  std::vector<int> ids() const { std::lock_guard<std::mutex> l(lock_); std::vector<int> r; for(const auto& p:bank_) r.push_back(p.first); return r; }

  void hold() { std::lock_guard<std::mutex> l(lock_); hold_unlocked(); }
  void position(const Vec& target,double yaw,double stamp=-1) {
    (void)stamp; // One-shot hold commands may share the release clock tick.
    require_vector(target); require_finite(yaw);
    std::lock_guard<std::mutex> l(lock_);
    if(landing_) return;
    reset(Mode::Position);
    reference_={}; reference_.position=target;
    reference_.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Vec::UnitZ()));
    setpoint_=reference_;
  }
  void timed(const Vec& target,double yaw,double seconds,bool landing=false) {
    require_vector(target); require_finite(yaw); require_positive(seconds);
    std::lock_guard<std::mutex> l(lock_);
    reset(Mode::Timed);
    landing_=landing;
    from_=measured_; to_.position=target;
    to_.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(yaw,Vec::UnitZ()));
    start_=now_; seconds_=seconds;
  }
  void full_state(const Reference& r,double stamp=-1) {
    require_vector(r.position); require_vector(r.velocity); require_vector(r.acceleration); require_vector(r.omega);
    if(!r.orientation.coeffs().allFinite() || !std::isfinite(r.orientation.norm()) || r.orientation.norm()<1e-9) throw std::invalid_argument("Invalid quaternion");
    std::lock_guard<std::mutex> l(lock_);
    if(landing_ || (stamp>=0 && (stamp<=stream_cutoff_ || now_-stamp>0.5))) return;
    if(mode_!=Mode::FullState) reset(Mode::FullState);
    release_=-1;
    reference_=r; reference_.orientation.normalize(); received_=now_;
  }
  void hover(double vx,double vy,double yaw_rate,double z,double stamp=-1) {
    require_finite(vx); require_finite(vy); require_finite(yaw_rate); require_finite(z);
    std::lock_guard<std::mutex> l(lock_);
    if(landing_ || (stamp>=0 && stamp<=stream_cutoff_)) return;
    if(mode_!=Mode::Hover) reset(Mode::Hover);
    release_=-1;
    hover_=Vec(vx,vy,z); yaw_rate_=yaw_rate; received_=now_;
  }
  void release(double milliseconds) {
    if(!std::isfinite(milliseconds) || milliseconds<0) throw std::invalid_argument("Invalid validity");
    std::lock_guard<std::mutex> l(lock_);
    stream_cutoff_=now_;
    if(milliseconds==0) hold_unlocked();
    else if(mode_==Mode::FullState || mode_==Mode::Hover) release_=now_+milliseconds/1000;
  }
  void upload(int id,size_t offset,const std::vector<Piece>& pieces) {
    if(id<0 || id>255 || pieces.empty() || pieces.size()>4096) throw std::invalid_argument("Invalid trajectory id/count");
    for(const auto& p:pieces) {
      require_positive(p.duration);
      for(const auto* c:{&p.x,&p.y,&p.z,&p.yaw}) for(double x:*c) require_finite(x);
    }
    std::lock_guard<std::mutex> l(lock_);
    auto found=bank_.find(id);
    const size_t size=found==bank_.end() ? 0 : found->second.size();
    if(offset>size || offset+pieces.size()>4096) throw std::invalid_argument("piece_offset creates a gap or exceeds 4096");
    auto copy=found==bank_.end() ? std::vector<Piece>() : found->second;
    copy.resize(std::max(copy.size(),offset+pieces.size()));
    std::copy(pieces.begin(),pieces.end(),copy.begin()+offset);
    bank_[id]=std::move(copy);
  }
  void start(int id,double scale,bool reverse,bool relative,double start_time) {
    require_positive(scale); require_finite(start_time);
    std::lock_guard<std::mutex> l(lock_);
    auto p=bank_.find(id);
    if(p==bank_.end() || p->second.empty()) throw std::invalid_argument("Trajectory not uploaded");
    // Copy before reset: uploads never mutate the active execution snapshot.
    active_=p->second; scale_=scale; reverse_=reverse; trajectory_origin_=setpoint_;
    reset(Mode::Trajectory);
    actual_start_=-1;
    start_=start_time;
    auto first=evaluate(active_,0,scale_,reverse_);
    offset_.setZero();
    if(relative) offset_=trajectory_origin_.position-first.position;
  }
  Output tick(double now,const Pose& pose) {
    std::lock_guard<std::mutex> l(lock_);
    if(initialized_ && now<now_-0.001) {
      now_=now; measured_=pose; hold_unlocked(); stream_cutoff_=-1; reason_="simulation_reset";
      return {};
    }
    double dt=std::clamp(now-now_,0.0,0.1);
    now_=now; measured_=pose;
    if(!initialized_) { initialized_=true; hold_unlocked(); }
    if((mode_==Mode::FullState || mode_==Mode::Hover) && (now_-received_>0.5 || (release_>=0 && now_>=release_))) {
      hold_unlocked(); reason_="stream_timeout";
    }
    Reference r=reference_;
    if(mode_==Mode::Timed) {
      double u=std::clamp((now_-start_)/seconds_,0.0,1.0);
      double blend=u*u*u*(10+u*(-15+6*u));
      double d=30*u*u*(1-u)*(1-u)/seconds_;
      r.position=from_.position+blend*(to_.position-from_.position);
      r.velocity=d*(to_.position-from_.position);
      r.orientation=from_.orientation.slerp(blend,to_.orientation);
      if(u>=1) { reference_=r; reference_.velocity.setZero(); mode_=Mode::Position; landing_=false; }
    } else if(mode_==Mode::Trajectory) {
      if(now_<start_) { r=trajectory_origin_; }
      else {
        if(actual_start_<0) actual_start_=now_;
        r=evaluate(active_,now_-start_,scale_,reverse_); r.position+=offset_;
        if(now_-start_>=duration(active_)*scale_) {
          r.velocity.setZero(); r.acceleration.setZero(); r.omega.setZero();
          reference_=r; mode_=Mode::Position;
        }
      }
    } else if(mode_==Mode::FullState) {
      double age=std::clamp(now_-received_,0.0,0.5);
      r.position+=r.velocity*age+0.5*r.acceleration*age*age;
      r.velocity+=r.acceleration*age;
      if(r.omega.norm()>1e-9)
        r.orientation=(r.orientation*Eigen::Quaterniond(Eigen::AngleAxisd(r.omega.norm()*age,r.omega.normalized()))).normalized();
    }
    Output out;
    if(mode_==Mode::Hover) {
      double yaw=std::atan2(pose.orientation.toRotationMatrix()(1,0),pose.orientation.toRotationMatrix()(0,0));
      out.linear=Eigen::AngleAxisd(yaw,Vec::UnitZ())*Vec(hover_.x(),hover_.y(),0);
      const double error=hover_.z()-pose.position.z();
      integral_.z()=std::clamp(integral_.z()+error*dt,-2.0,2.0);
      out.linear.z()=error+0.1*integral_.z();
      out.angular.z()=yaw_rate_;
      setpoint_.position=pose.position; setpoint_.position.z()=hover_.z();
      setpoint_.orientation=pose.orientation;
      return bounded(out);
    }
    if(!r.position.allFinite() || !r.velocity.allFinite() || !r.acceleration.allFinite()
        || !r.omega.allFinite() || !r.orientation.coeffs().allFinite()) {
      hold_unlocked(); reason_="nonfinite_reference"; return {};
    }
    Vec error=r.position-pose.position;
    integral_=(integral_+error*dt).cwiseMax(-2).cwiseMin(2);
    out.linear=r.velocity+error+0.1*integral_;
    Eigen::Quaterniond difference=pose.orientation.conjugate()*r.orientation;
    if(difference.w()<0) difference.coeffs()*=-1;
    Eigen::AngleAxisd axis(difference.normalized());
    Vec body_rate=r.omega+axis.axis()*axis.angle();
    out.angular=pose.orientation*body_rate;
    setpoint_=r;
    if(pose.position.z()<0.025 && r.position.z()<=0.025) { out.linear.z()=0; integral_.z()=0; }
    if(!out.linear.allFinite() || !out.angular.allFinite()) { hold_unlocked(); reason_="nonfinite_output"; return {}; }
    return bounded(out);
  }
 private:
  Output bounded(Output out) {
    if(!out.linear.allFinite() || !out.angular.allFinite()) {
      hold_unlocked(); reason_="nonfinite_output"; return {};
    }
    auto limit=[](Vec& v,double maximum) {
      double component=v.cwiseAbs().maxCoeff();
      if(component==0) return false;
      Vec scaled=v/component;
      double length=scaled.norm();
      if(component>maximum/length) { v=scaled*(maximum/length); return true; }
      return false;
    };
    bool linear=limit(out.linear,3.0), angular=limit(out.angular,6.0);
    if(linear || angular) reason_="velocity_limit";
    else if(reason_=="velocity_limit") reason_.clear();
    return out;
  }
  static void require_finite(double x) { if(!std::isfinite(x)) throw std::invalid_argument("Non-finite value"); }
  static void require_positive(double x) { require_finite(x); if(x<=0) throw std::invalid_argument("Duration/timescale must be positive"); }
  static void require_vector(const Vec& v) { if(!v.allFinite()) throw std::invalid_argument("Invalid vector"); }
  void reset(Mode mode) { mode_=mode; release_=-1; integral_.setZero(); reason_.clear(); landing_=false; }
  void hold_unlocked() { reset(Mode::Hold); reference_={}; reference_.position=measured_.position; reference_.orientation=measured_.orientation; setpoint_=reference_; }
  mutable std::mutex lock_;
  Mode mode_=Mode::Hold;
  Pose measured_,from_,to_;
  Reference reference_,setpoint_,trajectory_origin_;
  Vec integral_=Vec::Zero(),hover_=Vec::Zero(),offset_=Vec::Zero();
  double now_=0,start_=0,seconds_=1,received_=0,release_=-1,scale_=1,yaw_rate_=0;
  double actual_start_=-1;
  bool initialized_=false,reverse_=false;
  bool landing_=false;
  double stream_cutoff_=-1;
  unsigned group_=0;
  std::string reason_;
  std::map<int,std::vector<Piece>> bank_;
  std::vector<Piece> active_;
};
} // namespace ds_motion
