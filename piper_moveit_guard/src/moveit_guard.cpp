#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <sstream>

using JS = sensor_msgs::msg::JointState;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Validity = moveit_msgs::srv::GetStateValidity;

class Guard : public rclcpp::Node {
  struct Arm { std::array<double, 7> q{}; Time received{}; bool valid=false; };
  std::array<Arm,2> feedback_, target_, hold_;
  moveit::core::RobotModelPtr model_;
  std::unique_ptr<planning_scene::PlanningScene> scene_;
  std::vector<rclcpp::Subscription<JS>::SharedPtr> subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stage1_;
  std::array<rclcpp::Publisher<JS>::SharedPtr,2> output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_;
  rclcpp::Publisher<JS>::SharedPtr accepted_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_;
  rclcpp::Service<Validity>::SharedPtr validity_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool enforce_, demo_, confirmed_, latched_=false, holding_=false, upstream_stop_=true;
  Time upstream_time_{};
  Time started_=Clock::now();
  double timeout_, skew_, step_, grip_step_, budget_;
  int max_samples_;
  std::string reason_="waiting_for_inputs", pair_;
  Eigen::Vector3d contact_position_=Eigen::Vector3d::Zero();
  double penetration_=0.;
  static std::string side(int i) {return i==0 ? "left" : "right";}
  static double elapsed(Time t) { return std::chrono::duration<double>(Clock::now()-t).count(); }
  void trip(const std::string& why) {latched_=true; reason_=why;}

  bool fresh(const std::array<Arm,2>& arms) const {
    return arms[0].valid && arms[1].valid && elapsed(arms[0].received)<=timeout_ &&
      elapsed(arms[1].received)<=timeout_ &&
      std::abs(std::chrono::duration<double>(arms[0].received-arms[1].received).count())<=skew_;
  }
  bool upstreamClear() const {return !upstream_stop_ && elapsed(upstream_time_)<=timeout_;}
  std::array<double,16> positions(const std::array<Arm,2>& arms) const {
    std::array<double,16> values{};
    for(int s=0;s<2;++s) {
      for(int j=0;j<6;++j) values[s*8+j]=arms[s].q[j];
      values[s*8+6]=arms[s].q[6]/2.; values[s*8+7]=-arms[s].q[6]/2.;
    }
    return values;
  }
  std::vector<std::string> names() const {
    std::vector<std::string> out;
    for(int s=0;s<2;++s) for(int j=1;j<=8;++j) out.push_back(side(s)+"_joint"+std::to_string(j));
    return out;
  }
  bool collision(const std::array<double,16>& q, std::vector<moveit_msgs::msg::ContactInformation>* contacts=nullptr) {
    moveit::core::RobotState state(model_); state.setToDefaultValues();
    const auto ns=names();
    for(size_t i=0;i<q.size();++i) {
      if(!std::isfinite(q[i])) throw std::runtime_error("nonfinite_state");
      state.setVariablePosition(ns[i],q[i]);
    }
    if(!state.satisfiesBounds()) throw std::runtime_error("state_out_of_bounds");
    state.update();
    collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=1;
    collision_detection::CollisionResult result;
    scene_->checkSelfCollision(req,result,state);
    pair_.clear();penetration_=0.;
    if(!result.contacts.empty()) {
      pair_=result.contacts.begin()->first.first+":"+result.contacts.begin()->first.second;
      const auto& contact=result.contacts.begin()->second.front();
      contact_position_=contact.pos;penetration_=std::max(0.,contact.depth);
    }
    if(contacts) for(const auto& entry:result.contacts) for(const auto& contact:entry.second) {
      moveit_msgs::msg::ContactInformation c;c.header.frame_id="world";c.header.stamp=now();
      c.contact_body_1=entry.first.first;c.contact_body_2=entry.first.second;
      c.position.x=contact.pos.x();c.position.y=contact.pos.y();c.position.z=contact.pos.z();
      c.normal.x=contact.normal.x();c.normal.y=contact.normal.y();c.normal.z=contact.normal.z();
      c.depth=contact.depth;contacts->push_back(c);
    }
    return result.collision;
  }
  bool pathSafe(Time started) {
    const auto a=positions(feedback_), b=positions(target_);
    std::array<int,2> n{1,1};
    for(int s=0;s<2;++s) for(int j=0;j<8;++j) {
      const double delta=std::abs(b[s*8+j]-a[s*8+j]);
      n[s]=std::max(n[s],static_cast<int>(std::ceil(delta/(j<6?step_:grip_step_))));
    }
    // Check independent progress of the arms, including either arm held still.
    // This is a sampled grid, not a continuous swept-volume guarantee.
    if(static_cast<int64_t>(n[0]+1)*(n[1]+1)>max_samples_) {trip("path_sample_budget");return false;}
    for(int l=0;l<=n[0];++l) for(int r=0;r<=n[1];++r) {
      if(elapsed(started)>budget_) {trip("calculation_timeout");return false;}
      auto q=a;
      for(int s=0;s<2;++s) for(int j=0;j<8;++j) {
        double t=static_cast<double>(s==0?l:r)/n[s];
        q[s*8+j]=a[s*8+j]+t*(b[s*8+j]-a[s*8+j]);
      }
      if(collision(q)) {trip("predicted_box_collision");return false;}
    }
    return true;
  }
  void receive(const JS& msg,int s,bool target) {
    auto& arm=target?target_[s]:feedback_[s]; arm.valid=false;
    try {
      double age=(now()-rclcpp::Time(msg.header.stamp)).seconds();
      if(rclcpp::Time(msg.header.stamp).nanoseconds()<=0 || age < -0.05 || age>timeout_)
        throw std::runtime_error("source_timestamp");
      if(msg.name.size()!=msg.position.size()) throw std::runtime_error("malformed_state");
      std::set<std::string> unique(msg.name.begin(),msg.name.end());
      if(unique.size()!=msg.name.size()) throw std::runtime_error("duplicate_joints");
      for(int j=0;j<7;++j) {
        const std::string local=j<6?"joint"+std::to_string(j+1):"gripper";
        auto it=std::find(msg.name.begin(),msg.name.end(),local);
        if(it==msg.name.end()) it=std::find(msg.name.begin(),msg.name.end(),side(s)+"_"+local);
        if(it==msg.name.end() && j==6) it=std::find(msg.name.begin(),msg.name.end(),"joint7");
        if(it==msg.name.end()) throw std::runtime_error("missing_joint");
        arm.q[j]=msg.position[std::distance(msg.name.begin(),it)];
        if(!std::isfinite(arm.q[j])) throw std::runtime_error("nonfinite_state");
        if(j<6) {
          const auto& bounds=model_->getVariableBounds(side(s)+"_joint"+std::to_string(j+1));
          if(arm.q[j]<bounds.min_position_ || arm.q[j]>bounds.max_position_) throw std::runtime_error("joint_bounds");
        } else if(arm.q[j]<0 || arm.q[j]>.07) throw std::runtime_error("gripper_bounds");
      }
      arm.received=Clock::now()-std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(std::max(0.,age)));
      arm.valid=true;
    } catch(const std::exception&) {trip(target?"invalid_target":"invalid_feedback");}
  }
  void publish(const std::array<Arm,2>& arms) {
    for(int s=0;s<2;++s) {
      JS msg; msg.header.stamp=now();
      for(int j=1;j<=6;++j) msg.name.push_back("joint"+std::to_string(j));
      msg.name.push_back("gripper"); msg.position.assign(arms[s].q.begin(),arms[s].q.end());
      msg.velocity.assign(7,0.);msg.velocity[6]=get_parameter("driver_speed_ratio").as_int();
      msg.effort.assign(7,0.);msg.effort[6]=get_parameter("gripper_effort_nm").as_double();
      output_[s]->publish(msg);
    }
  }
  void report(bool blocked,bool pose_collision=false) {
    std_msgs::msg::Bool stop;stop.data=blocked;stop_->publish(stop);
    std_msgs::msg::String msg;std::ostringstream os;
    os<<"{\"mode\":\""<<(demo_?"demo":enforce_?"enforce":"monitor")<<"\",\"blocked\":"<<(blocked?"true":"false")
      <<",\"latched\":"<<(latched_?"true":"false")<<",\"pose_collision\":"<<(pose_collision?"true":"false")
      <<",\"penetration_depth_m\":"<<penetration_<<",\"reason\":\""<<reason_<<"\",\"pair\":\""<<pair_<<"\"}";
    msg.data=os.str();status_->publish(msg);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id="world";marker.header.stamp=now();marker.ns="moveit_status";marker.id=0;
    marker.type=visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action=visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z=.65;marker.pose.orientation.w=1.;marker.scale.z=.035;
    marker.color.r=blocked?1.f:.2f;marker.color.g=blocked?.2f:1.f;marker.color.b=.2f;marker.color.a=1.;
    marker.text=(blocked?"BLOCKED: ":"CLEAR: ")+reason_+"\n"+pair_;
    if(!pair_.empty()) marker.text+="\nBox overlap: "+std::to_string(penetration_*1000.)+" mm";
    marker_->publish(marker);
    marker.ns="moveit_contact";marker.id=1;marker.type=visualization_msgs::msg::Marker::SPHERE;
    marker.action=pair_.empty()?visualization_msgs::msg::Marker::DELETE:visualization_msgs::msg::Marker::ADD;
    marker.text.clear();marker.pose.position.x=contact_position_.x();
    marker.pose.position.y=contact_position_.y();marker.pose.position.z=contact_position_.z();
    marker.scale.x=marker.scale.y=marker.scale.z=.018;
    marker.color.r=1.;marker.color.g=0.;marker.color.b=0.;marker_->publish(marker);
  }
  void tick() {
    const auto started=Clock::now(); bool safe=false, current_collision=false;
    try {
      if(!fresh(feedback_)) {
        if(elapsed(started_)>timeout_) trip("feedback_timeout_or_skew");
        else reason_="waiting_for_feedback";
      }
      else {
        current_collision=collision(positions(feedback_));
        if(current_collision) trip("current_box_collision");
        if(!latched_ && fresh(target_) && upstreamClear()) safe=pathSafe(started);
        else if(!latched_) reason_="waiting_for_stage1_or_targets";
      }
      // Check again after the expensive computation: never forward an old snapshot.
      if(safe && (!fresh(feedback_) || !fresh(target_) || !upstreamClear() || elapsed(started)>budget_)) {
        safe=false;trip("inputs_expired_during_check");
      }
      if(enforce_ && (count_publishers("/joint_ctrl_cmd_left")>1 || count_publishers("/joint_ctrl_cmd_right")>1)) {
        safe=false;trip("another_driver_command_publisher");
      }
      if(safe && !latched_) {
        reason_="clear";holding_=false;
        JS accepted;accepted.header.stamp=now();accepted.name=names();auto p=positions(target_);
        accepted.position.assign(p.begin(),p.end());accepted_->publish(accepted);
        if(enforce_) publish(target_);
      } else if(enforce_ && fresh(feedback_)) {
        if(!holding_) {hold_=feedback_;holding_=true;}
        publish(hold_); // Position hold request, not a hardware emergency stop.
      }
    } catch(const std::exception&) {trip("collision_calculation_error");safe=false;}
    report(!safe || latched_,current_collision);
  }
public:
  Guard():Node("piper_moveit_guard") {
    enforce_=declare_parameter("enforce",false);demo_=declare_parameter("demo",false);
    confirmed_=declare_parameter("geometry_confirmed",false);
    timeout_=declare_parameter("input_timeout_s",0.15);skew_=declare_parameter("max_arm_skew_s",0.05);
    step_=declare_parameter("path_step_rad",0.05);grip_step_=declare_parameter("gripper_step_m",0.002);
    budget_=declare_parameter("calculation_budget_s",0.05);max_samples_=declare_parameter("max_path_samples",2500);
    const auto speed=declare_parameter("driver_speed_ratio",10);
    const auto effort=declare_parameter("gripper_effort_nm",1.0);
    if((enforce_ && (!confirmed_ || demo_)) || !std::isfinite(timeout_) || timeout_<=0 ||
       !std::isfinite(skew_) || skew_<0 || !std::isfinite(step_) || step_<=0 ||
       !std::isfinite(grip_step_) || grip_step_<=0 || !std::isfinite(budget_) || budget_<=0 ||
       max_samples_<4 || speed<1 || speed>100 || !std::isfinite(effort) || effort<.5 || effort>3.)
      throw std::runtime_error("Invalid guard configuration; enforce requires confirmed geometry and demo=false");
  }
  void init() {
    robot_model_loader::RobotModelLoader loader(shared_from_this(),"robot_description",false);
    model_=loader.getModel();if(!model_) throw std::runtime_error("No robot model");
    scene_=std::make_unique<planning_scene::PlanningScene>(model_);
    // Assert all 100 cross-arm pairs are checked, regardless of supplied SRDF.
    auto& acm=scene_->getAllowedCollisionMatrixNonConst();
    for(const auto* a:model_->getLinkModelsWithCollisionGeometry())
      for(const auto* b:model_->getLinkModelsWithCollisionGeometry())
        if(a->getName().rfind("left_",0)==0 && b->getName().rfind("right_",0)==0)
          acm.setEntry(a->getName(),b->getName(),false);
    const auto latched=rclcpp::QoS(1).reliable().transient_local();
    status_=create_publisher<std_msgs::msg::String>("/moveit_guard/status",latched);
    stop_=create_publisher<std_msgs::msg::Bool>("/moveit_guard/stop_requested",latched);
    marker_=create_publisher<visualization_msgs::msg::Marker>("/moveit_guard/marker",10);
    accepted_=create_publisher<JS>("/moveit_guard/accepted_joint_states",1);
    validity_=create_service<Validity>("/moveit_guard/check_state_validity",
      [this](const Validity::Request::SharedPtr req,Validity::Response::SharedPtr res) {
        res->valid=false;
        try {
          const auto& msg=req->robot_state.joint_state;
          if(msg.name.size()!=16 || msg.position.size()!=16) return;
          std::set<std::string> unique(msg.name.begin(),msg.name.end());if(unique.size()!=16) return;
          std::array<double,16> q{};const auto ns=names();
          for(size_t i=0;i<ns.size();++i) {
            auto it=std::find(msg.name.begin(),msg.name.end(),ns[i]);if(it==msg.name.end()) return;
            q[i]=msg.position[std::distance(msg.name.begin(),it)];
          }
          res->valid=!collision(q,&res->contacts);
        } catch(const std::exception&) {res->valid=false;}
      });
    if(demo_) {
      subs_.push_back(create_subscription<JS>("/moveit_guard/demo_joint_states",1,[this](JS::SharedPtr msg){
        // Full state, exact names required.
        std::array<double,16> q{};const auto ns=names();
        bool valid=msg->name.size()==msg->position.size();
        std::set<std::string> unique(msg->name.begin(),msg->name.end());valid &= unique.size()==msg->name.size();
        for(size_t i=0;i<ns.size() && valid;++i) {
          auto it=std::find(msg->name.begin(),msg->name.end(),ns[i]);
          if(it==msg->name.end()) {valid=false;break;}
          q[i]=msg->position[std::distance(msg->name.begin(),it)];
        }
        try {
          if(!valid) throw std::runtime_error("invalid_demo_state");
          bool hit=collision(q);reason_=hit?"box_collision":"clear";report(hit,hit);
        } catch(const std::exception&) {reason_="invalid_demo_state";report(true);}
      }));
      RCLCPP_INFO(get_logger(),"DEMO: inspect poses only; no driver publishers");return;
    }
    for(int s=0;s<2;++s) {
      subs_.push_back(create_subscription<JS>("/joint_states_"+side(s),1,[this,s](JS::SharedPtr m){receive(*m,s,false);}));
      subs_.push_back(create_subscription<JS>(enforce_?"/pinocchio_checked/"+side(s)+"_target":"/teleop/"+side(s)+"_target",1,
        [this,s](JS::SharedPtr m){receive(*m,s,true);}));
      if(enforce_) output_[s]=create_publisher<JS>("/joint_ctrl_cmd_"+side(s),1);
    }
    stage1_=create_subscription<std_msgs::msg::Bool>("/collision_guard/stop_requested",latched,
      [this](std_msgs::msg::Bool::SharedPtr msg){upstream_stop_=msg->data;upstream_time_=Clock::now();});
    reset_=create_service<std_srvs::srv::Trigger>("/moveit_guard/reset",[this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response){
      try {
        response->success=fresh(feedback_) && !collision(positions(feedback_));
        if(response->success) {latched_=false;holding_=false;target_={};reason_="reset_waiting_for_new_targets";}
      } catch(const std::exception&) {response->success=false;}
      response->message=response->success?"Reset; fresh stage-1 commands required":"Reset refused: stale feedback or box collision";
    });
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    RCLCPP_INFO(get_logger(),"%s: cross-arm MoveIt/FCL box checks",enforce_?"ENFORCE":"MONITOR");
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {auto node=std::make_shared<Guard>();node->init();rclcpp::spin(node);}
  catch(const std::exception& e) {RCLCPP_FATAL(rclcpp::get_logger("piper_moveit_guard"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
