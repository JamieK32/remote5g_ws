#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <remote5g_sim/control_math.hpp>
#include <chrono>
#include <functional>
#include <mutex>
#include <map>
#include <string>
#include <vector>

namespace remote5g {
class VehiclePlugin final : public gazebo::ModelPlugin {
  using Clock=std::chrono::steady_clock;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr node_;
  std::vector<gazebo::physics::JointPtr> joints_;
  std::vector<std::string> names_{"front_left_steering_joint","front_right_steering_joint",
    "rear_left_wheel_joint","rear_right_wheel_joint","front_left_wheel_joint",
    "front_right_wheel_joint","gimbal_pan_joint","gimbal_tilt_joint"};
  std::map<std::string,double> p_;
  std::mutex mutex_;
  double requested_speed_=0, requested_angle_=0, speed_=0, angle_=0;
  double pan_=0, tilt_=0, pan_goal_=0, tilt_goal_=0;
  double accel_=0, rate_=0, last_control_=0, last_pub_=0, last_time_=0;
  bool valid_=false, fault_=false;
  std::string state_="startup_stopped";
  Clock::time_point lease_=Clock::now();
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ack_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pan_sub_, tilt_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr center_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::vector<double> effort_=std::vector<double>(8,0);
  void stop(const std::string &why) {
    valid_=false; requested_speed_=0; requested_angle_=0;
    if(state_!=why) RCLCPP_WARN(node_->get_logger(), "Safe stop: %s",why.c_str());
    state_=why;
  }
  void command(double v, double a, double accel, double rate) {
    if(fault_) return;
    if(!std::isfinite(v)||!std::isfinite(a)||!std::isfinite(accel)||!std::isfinite(rate)||accel<0||rate<0) {
      stop("invalid_command"); return;
    }
    requested_angle_=clamp(a,-p_["max_steering"],p_["max_steering"]);
    double curvature=std::tan(requested_angle_)/p_["wheelbase"];
    double cap=clamp(p_["max_speed"]/(1+p_["curvature_speed_gain"]*std::abs(curvature)),
      p_["min_curve_speed"],p_["max_speed"]);
    requested_speed_=clamp(v,-cap,cap);
    accel_=accel>0?std::min(accel,p_["max_accel"]):p_["max_accel"];
    rate_=rate>0?std::min(rate,p_["steering_rate"]):p_["steering_rate"];
    lease_=Clock::now(); valid_=true; state_="active";
  }
  double servo(size_t i,double goal,double kp,double kd,double limit) {
    double u=clamp(kp*(goal-joints_[i]->Position(0))-kd*joints_[i]->GetVelocity(0),-limit,limit);
    joints_[i]->SetForce(0,u); effort_[i]=u; return u;
  }
  void publish(const gazebo::common::Time &time) {
    builtin_interfaces::msg::Time stamp; stamp.sec=time.sec; stamp.nanosec=time.nsec;
    // Actual simulated link pose, shifted to rear axle ground reference. No kinematic integration.
    auto pose=base_->WorldPose();
    auto offset=pose.Rot().RotateVector(ignition::math::Vector3d(0,0,p_["wheel_radius"]));
    pose.Pos()-=offset;
    auto vel=base_->WorldLinearVel()-base_->WorldAngularVel().Cross(offset);
    auto local=pose.Rot().RotateVectorReverse(vel);
    nav_msgs::msg::Odometry o; o.header.stamp=stamp; o.header.frame_id="odom"; o.child_frame_id="base_footprint";
    o.pose.pose.position.x=pose.Pos().X(); o.pose.pose.position.y=pose.Pos().Y(); o.pose.pose.position.z=pose.Pos().Z();
    o.pose.pose.orientation.x=pose.Rot().X(); o.pose.pose.orientation.y=pose.Rot().Y();
    o.pose.pose.orientation.z=pose.Rot().Z(); o.pose.pose.orientation.w=pose.Rot().W();
    o.twist.twist.linear.x=local.X(); o.twist.twist.linear.y=local.Y(); o.twist.twist.linear.z=local.Z();
    auto angular=pose.Rot().RotateVectorReverse(base_->WorldAngularVel());
    o.twist.twist.angular.x=angular.X(); o.twist.twist.angular.y=angular.Y(); o.twist.twist.angular.z=angular.Z();
    // Ground truth estimator: very small numerical uncertainty, not a hardware odometer.
    for(int i: {0,7,14,21,28,35}) { o.pose.covariance[i]=1e-6; o.twist.covariance[i]=1e-6; }
    odom_pub_->publish(o);
    geometry_msgs::msg::TransformStamped t; t.header=o.header; t.child_frame_id=o.child_frame_id;
    t.transform.translation.x=pose.Pos().X(); t.transform.translation.y=pose.Pos().Y(); t.transform.translation.z=pose.Pos().Z();
    t.transform.rotation=o.pose.pose.orientation; tf_->sendTransform(t);
    sensor_msgs::msg::JointState j; j.header.stamp=stamp; j.name=names_; j.effort=effort_;
    for(auto &joint:joints_) { j.position.push_back(joint->Position(0)); j.velocity.push_back(joint->GetVelocity(0)); }
    joint_pub_->publish(j);
    diagnostic_msgs::msg::DiagnosticArray d; d.header.stamp=stamp;
    diagnostic_msgs::msg::DiagnosticStatus s; s.name="XT-NetRC virtual actuators"; s.hardware_id="gazebo_only";
    s.level=fault_?2:(valid_?0:1); s.message=state_;
    auto value=[&](std::string key,double v) { diagnostic_msgs::msg::KeyValue x; x.key=key; x.value=std::to_string(v); s.values.push_back(x); };
    value("speed_setpoint",speed_); value("steering_setpoint",angle_);
    value("requested_speed",requested_speed_); value("requested_angle",requested_angle_);
    value("common_rear_torque",effort_[2]); value("lease_age_s",std::chrono::duration<double>(Clock::now()-lease_).count());
    d.status.push_back(s); diag_pub_->publish(d);
  }
  void update(const gazebo::common::UpdateInfo &info) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now=info.simTime.Double();
    if(now<last_time_) { stop("simulation_reset"); speed_=angle_=pan_=tilt_=pan_goal_=tilt_goal_=0; last_control_=last_pub_=now; }
    last_time_=now;
    try {
      if(valid_ && std::chrono::duration<double>(Clock::now()-lease_).count()>p_["command_timeout"]) stop("command_timeout");
      if(now-last_control_>=p_["control_period"]) {
        const double dt=std::min(now-last_control_,2*p_["control_period"]);
        speed_=speed_step(speed_,requested_speed_,accel_,p_["max_decel"],dt);
        angle_=approach(angle_,requested_angle_,rate_*dt);
        pan_=approach(pan_,pan_goal_,p_["gimbal_rate"]*dt);
        tilt_=approach(tilt_,tilt_goal_,p_["gimbal_rate"]*dt);
        last_control_=now;
      }
      auto a=ackermann(angle_,p_["wheelbase"],p_["front_track"]);
      servo(0,a.first,p_["steering_kp"],p_["steering_kd"],p_["steering_torque"]);
      servo(1,a.second,p_["steering_kp"],p_["steering_kd"],p_["steering_torque"]);
      // One virtual motor + open rear differential: identical torque on both half shafts.
      // Wheel angular speeds emerge from contact constraints; no left/right speed commands.
      double actual=(joints_[2]->GetVelocity(0)+joints_[3]->GetVelocity(0))*0.5*p_["wheel_radius"];
      double torque=clamp(p_["drive_kp"]*(speed_-actual),-p_["drive_torque"],p_["drive_torque"]);
      for(int i: {2,3}) { joints_[i]->SetForce(0,torque); effort_[i]=torque; }
      servo(6,pan_,p_["gimbal_kp"],p_["gimbal_kd"],p_["gimbal_torque"]);
      servo(7,tilt_,p_["gimbal_kp"],p_["gimbal_kd"],p_["gimbal_torque"]);
      if(now-last_pub_>=p_["control_period"]) { publish(info.simTime); last_pub_=now; }
    } catch(const std::exception &e) {
      stop("controller_exception"); fault_=true; speed_=0; angle_=0;
      for(int i: {2,3}) joints_[i]->SetForce(0,0);
      RCLCPP_ERROR(node_->get_logger(),"Controller fault: %s",e.what());
    }
  }
public:
  void Load(gazebo::physics::ModelPtr model,sdf::ElementPtr sdf) override {
    model_=model; node_=gazebo_ros::Node::Get(sdf); base_=model_->GetLink("base_link");
    if(!base_) throw std::runtime_error("base_link absent");
    for(const auto &name:names_) { auto j=model_->GetJoint(name); if(!j) throw std::runtime_error("missing joint: "+name); joints_.push_back(j); }
    for(auto key: {"wheelbase","front_track","wheel_radius","max_steering","joint_steering_limit","max_speed","steering_rate","max_accel","max_decel","curvature_speed_gain","min_curve_speed","command_timeout","control_period","steering_kp","steering_kd","steering_torque","drive_kp","drive_torque","pan_limit","tilt_min","tilt_max","gimbal_rate","gimbal_kp","gimbal_kd","gimbal_torque"}) {
      if(!sdf->HasElement(key)) throw std::runtime_error(std::string("missing parameter: ")+key);
      double v=sdf->Get<double>(key); if(!std::isfinite(v)) throw std::runtime_error("nonfinite configuration"); p_[key]=v;
    }
    for(auto &entry:p_) if(entry.first!="tilt_min" && entry.second<=0) throw std::runtime_error("nonpositive configuration: "+entry.first);
    if(p_["max_steering"]>=1.0 || p_["tilt_min"]>=0 || p_["tilt_max"]<=0 || p_["min_curve_speed"]>p_["max_speed"] ||
      std::abs(ackermann(p_["max_steering"],p_["wheelbase"],p_["front_track"]).first)>p_["joint_steering_limit"])
      throw std::runtime_error("inconsistent steering/speed limits");
    accel_=p_["max_accel"]; rate_=p_["steering_rate"];
    ack_sub_=node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd",rclcpp::QoS(1),[this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_); auto &d=msg->drive;
      if(!std::isfinite(d.jerk)) { stop("invalid_command"); return; }
      command(d.speed,d.steering_angle,d.acceleration,d.steering_angle_velocity);
    });
    twist_sub_=node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(1),[this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_); double a=0;
      bool planar=std::isfinite(msg->linear.y)&&std::isfinite(msg->linear.z)&&std::isfinite(msg->angular.x)&&std::isfinite(msg->angular.y)&&
        std::abs(msg->linear.y)<1e-6&&std::abs(msg->linear.z)<1e-6&&std::abs(msg->angular.x)<1e-6&&std::abs(msg->angular.y)<1e-6;
      if(!planar||!twist(msg->linear.x,msg->angular.z,p_["wheelbase"],a)) { stop("unrealizable_twist"); return; }
      command(msg->linear.x,a,0,0);
    });
    auto gimbal=[this](bool pan,double x) { std::lock_guard<std::mutex> lock(mutex_);
      if(!std::isfinite(x)) { RCLCPP_WARN(node_->get_logger(),"Invalid gimbal command rejected"); return; }
      if(pan) pan_goal_=clamp(x,-p_["pan_limit"],p_["pan_limit"]); else tilt_goal_=clamp(x,p_["tilt_min"],p_["tilt_max"]); };
    pan_sub_=node_->create_subscription<std_msgs::msg::Float64>("gimbal/pan_cmd",1,[gimbal](const std_msgs::msg::Float64::SharedPtr msg){gimbal(true,msg->data);});
    tilt_sub_=node_->create_subscription<std_msgs::msg::Float64>("gimbal/tilt_cmd",1,[gimbal](const std_msgs::msg::Float64::SharedPtr msg){gimbal(false,msg->data);});
    center_=node_->create_service<std_srvs::srv::Trigger>("gimbal/center",[this](const std_srvs::srv::Trigger::Request::SharedPtr, const std_srvs::srv::Trigger::Response::SharedPtr response) {
      std::lock_guard<std::mutex> lock(mutex_); pan_goal_=tilt_goal_=0; response->success=true; response->message="Center targets set in radians"; });
    odom_pub_=node_->create_publisher<nav_msgs::msg::Odometry>("odom",10);
    joint_pub_=node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    diag_pub_=node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",10);
    tf_=std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    last_control_=last_pub_=last_time_=model_->GetWorld()->SimTime().Double();
    update_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&VehiclePlugin::update,this,std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),"XT-NetRC plugin loaded: physical Ackermann, common rear torque, wall-clock watchdog, no hardware backend");
  }
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if(node_) stop("simulation_reset");
    speed_=angle_=pan_=tilt_=pan_goal_=tilt_goal_=0;
  }
};
GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)
}
