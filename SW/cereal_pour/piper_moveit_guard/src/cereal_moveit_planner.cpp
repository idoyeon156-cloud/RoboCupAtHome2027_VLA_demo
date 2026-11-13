// Planning-only service. No driver or arm-command publishers exist here.
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <atomic>
#include <chrono>
#include <future>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>

using Plan = moveit_msgs::srv::GetMotionPlan;
using Validity = moveit_msgs::srv::GetStateValidity;
using namespace std::chrono_literals;

class CerealPlanner : public rclcpp::Node {
  planning_scene::PlanningScenePtr scene_;
  planning_pipeline::PlanningPipelinePtr pipeline_;
  rclcpp::CallbackGroup::SharedPtr constraint_group_;
  rclcpp::Client<Validity>::SharedPtr constraints_;
  rclcpp::Service<Plan>::SharedPtr plan_;
  rclcpp::Service<Validity>::SharedPtr validity_;
  std::atomic<bool> constraint_failed_{false};
  std::atomic<size_t> checks_{0};
public:
  CerealPlanner():Node("cereal_moveit_planner") {
    declare_parameter<std::string>("service_prefix","/cereal_planner");
    declare_parameter<std::string>("planner_configs.RRTConnectkConfigDefault.type","geometric::RRTConnect");
    declare_parameter<double>("planner_configs.RRTConnectkConfigDefault.range",.15);
    for(const std::string group : {"left_arm","right_arm","both_arms"}) {
      declare_parameter<std::vector<std::string>>(group+".planner_configs",{"RRTConnectkConfigDefault"});
      declare_parameter<double>(group+".longest_valid_segment_fraction",.002);
      const auto side=group=="right_arm"?"right":"left";
      declare_parameter<std::string>(group+".projection_evaluator",std::string("joints(")+side+"_joint1,"+side+"_joint2)");
    }
  }
  void init() {
    const auto prefix=get_parameter("service_prefix").as_string();
    robot_model_loader::RobotModelLoader loader(shared_from_this(),"robot_description",false);
    auto model=loader.getModel();
    if(!model) throw std::runtime_error("No MoveIt robot model");
    scene_=std::make_shared<planning_scene::PlanningScene>(model);
    scene_->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
    auto& acm=scene_->getAllowedCollisionMatrixNonConst();
    for(const auto* a:model->getLinkModelsWithCollisionGeometry())
      for(const auto* b:model->getLinkModelsWithCollisionGeometry())
        if(a->getName().rfind("left_",0)==0 && b->getName().rfind("right_",0)==0)
          acm.setEntry(a->getName(),b->getName(),false);
    // Responses must run in a different callback group while the planning
    // callback waits. The Python constraint service computes the same
    // Pinocchio distance predicate and task-object constraints during search.
    constraint_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    constraints_=create_client<Validity>(prefix+"/constraints",rmw_qos_profile_services_default,constraint_group_);
    scene_->setStateFeasibilityPredicate([this](const moveit::core::RobotState& state,bool) {
      ++checks_;
      if(constraint_failed_.load()) return false;
      if(!constraints_->service_is_ready()) {
        constraint_failed_=true;return false;
      }
      auto request=std::make_shared<Validity::Request>();
      moveit::core::robotStateToRobotStateMsg(state,request->robot_state,false);
      auto future=constraints_->async_send_request(request);
      if(future.wait_for(2s)!=std::future_status::ready) {
        constraints_->remove_pending_request(future);
        constraint_failed_=true;return false;
      }
      try {return future.get()->valid;}
      catch(const std::exception&) {constraint_failed_=true;return false;}
    });
    pipeline_=std::make_shared<planning_pipeline::PlanningPipeline>(
      model,shared_from_this(),"","ompl_interface/OMPLPlanner",std::vector<std::string>{});
    pipeline_->displayComputedMotionPlans(false);
    pipeline_->publishReceivedRequests(false);
    pipeline_->checkSolutionPaths(true);
    plan_=create_service<Plan>(prefix+"/plan",[this](const Plan::Request::SharedPtr request,Plan::Response::SharedPtr response) {
      constraint_failed_=false;checks_=0;
      try {
        scene_->setCurrentState(request->motion_plan_request.start_state);
        planning_interface::MotionPlanResponse result;
        const bool success=pipeline_->generatePlan(scene_,request->motion_plan_request,result);
        result.getMessage(response->motion_plan_response);
        if(!success || constraint_failed_.load())
          response->motion_plan_response.error_code.val=moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        RCLCPP_INFO(get_logger(),"MoveIt plan: group=%s result=%d feasibility_checks=%zu constraint_transport_failed=%s",
          request->motion_plan_request.group_name.c_str(),response->motion_plan_response.error_code.val,
          checks_.load(),constraint_failed_.load()?"true":"false");
      } catch(const std::exception& error) {
        response->motion_plan_response.error_code.val=moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        RCLCPP_ERROR(get_logger(),"MoveIt planning failed: %s",error.what());
      }
    });
    validity_=create_service<Validity>(prefix+"/check_state",[this](const Validity::Request::SharedPtr request,Validity::Response::SharedPtr response) {
      constraint_failed_=false;
      try {
        moveit::core::RobotState state(scene_->getRobotModel());
        state.setToDefaultValues();
        moveit::core::robotStateMsgToRobotState(request->robot_state,state);
        state.update();
        collision_detection::CollisionRequest collision_request;
        collision_request.contacts=true;collision_request.max_contacts=10;
        collision_detection::CollisionResult collision_result;
        scene_->checkCollision(collision_request,collision_result,state);
        for(const auto& entry:collision_result.contacts) for(const auto& contact:entry.second) {
          moveit_msgs::msg::ContactInformation info;
          info.header.frame_id=scene_->getRobotModel()->getModelFrame();
          info.contact_body_1=entry.first.first;info.contact_body_2=entry.first.second;
          info.position.x=contact.pos.x();info.position.y=contact.pos.y();info.position.z=contact.pos.z();
          info.depth=std::abs(contact.depth);
          response->contacts.push_back(info);
        }
        response->valid=state.satisfiesBounds() && !collision_result.collision &&
          scene_->isStateFeasible(state) && !constraint_failed_.load();
      } catch(const std::exception&) {response->valid=false;}
    });
    RCLCPP_INFO(get_logger(),"Planning-only MoveIt OMPL/Bullet service ready; constraint service is mandatory");
  }
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<CerealPlanner>();node->init();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    executor.add_node(node);executor.spin();
  } catch(const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("cereal_moveit_planner"),"%s",error.what());
    rclcpp::shutdown();return 1;
  }
  rclcpp::shutdown();return 0;
}
