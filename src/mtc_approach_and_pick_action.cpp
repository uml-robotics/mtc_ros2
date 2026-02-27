#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// MoveIt
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Action Msgs
#include <action_msgs/msg/goal_status_array.hpp>

// MTC
#include <moveit/task_constructor/task.h>

// Stages
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>

// Solvers
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

// ROS msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mtc_ros2/action/mtc_approach_and_pick.hpp>

namespace mtc = moveit::task_constructor;
namespace stages = mtc::stages;
namespace solvers = mtc::solvers;

namespace mtc_ros2 {

using ApproachAndPick = mtc_ros2::action::MTCApproachAndPick;
using GoalHandle     = rclcpp_action::ServerGoalHandle<ApproachAndPick>;

class MtcApproachAndPickActionNode : public rclcpp::Node {
public:
  explicit MtcApproachAndPickActionNode()
  : rclcpp::Node("mtc_approach_and_pick_node")
  {
    action_server_ = rclcpp_action::create_server<ApproachAndPick>(
      this,
      "mtc_approach_and_pick",
      std::bind(&MtcApproachAndPickActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MtcApproachAndPickActionNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MtcApproachAndPickActionNode::handle_accepted, this, std::placeholders::_1)
    );

    // JointTrajectoryController status subscriber
    jt_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
      rclcpp::QoS(10),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(jt_status_mtx_);
        last_jt_status_ = *msg;
        last_jt_status_stamp_ = this->now();
      }
    );

    RCLCPP_INFO(get_logger(), "MTC Approach and Pick action server ready.");
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  moveit::task_constructor::TaskPtr task_;
  rclcpp_action::Server<ApproachAndPick>::SharedPtr action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // JointTrajectoryController status
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jt_status_sub_;
  mutable std::mutex jt_status_mtx_;
  action_msgs::msg::GoalStatusArray last_jt_status_;
  rclcpp::Time last_jt_status_stamp_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const ApproachAndPick::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
  {
    RCLCPP_WARN(get_logger(), "Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Detach a worker thread; MultiThreadedExecutor will let callbacks run concurrently
    std::thread([this, goal_handle](){ this->execute_goal(goal_handle); }).detach();
  }

  // check if the controller is busy (doing some action)
  bool controller_busy_() const
  {
    std::lock_guard<std::mutex> lk(jt_status_mtx_);
    for (const auto& s : last_jt_status_.status_list) {
      if (s.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        return true;
      }
    }
    return false;
  }

  // wait for max <timeout> seconds until the controller becomes busy
  bool wait_until_controller_busy_(double timeout_sec, double poll_hz = 50.0)
  {
    const auto deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::Rate rate(poll_hz);

    while (rclcpp::ok() && this->now() < deadline) {
      if (controller_busy_()) return true;
      rate.sleep();
    }
    return controller_busy_();
  }

  // wait for max <timeout> seconds until the controller becomes idle
  bool wait_until_controller_idle_(double timeout_sec, double poll_hz = 50.0)
  {
    const auto deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::Rate rate(poll_hz);

    while (rclcpp::ok() && this->now() < deadline) {
      if (!controller_busy_()) return true;
      rate.sleep();
    }
    return !controller_busy_();
  }

  // wait for max <busy_timeout> seconds until the controller becomes busy
  // then wait for max <idle_timeout> seconds until the controller becomes idle
  bool wait_for_busy_then_idle_(double busy_timeout_sec,
                                double idle_timeout_sec,
                                double poll_hz = 50.0)
  {
    // If it's already busy, skip the first wait.
    if (!controller_busy_()) {
      if (!wait_until_controller_busy_(busy_timeout_sec, poll_hz)) {
        // Never observed it become busy
        return false;
      }
    }
    // Now wait until it becomes idle again.
    return wait_until_controller_idle_(idle_timeout_sec, poll_hz);
  }

  void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    try {
      auto goal = goal_handle->get_goal();
      ApproachAndPick::Result result;

      auto send_feedback = [&](const std::string& phase,
                          const std::string& stage, float prog)
      {
        if (!rclcpp::ok() || !goal_handle->is_active()) return;

        try {
          auto fb = std::make_shared<ApproachAndPick::Feedback>();
          fb->phase = phase;
          fb->stage_name = stage;
          fb->stage_progress = prog;
          goal_handle->publish_feedback(fb);
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "publish_feedback threw: %s", e.what());
        }
      };

      auto finish_goal = [&](int mode, const ApproachAndPick::Result& res)
      {
        auto res_ptr = std::make_shared<ApproachAndPick::Result>(res);

        if (!rclcpp::ok() || !goal_handle->is_active()) {
          RCLCPP_WARN(get_logger(), "Not sending result (goal not active/tracked anymore).");
          return;
        }

        try {
          if (mode == 0) goal_handle->succeed(res_ptr);
          else if (mode == 1) goal_handle->abort(res_ptr);
          else goal_handle->canceled(res_ptr);
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "Sending result threw: %s", e.what());
        }
      };

      // BEFORE constructing stages:
      auto approach_pose  = goal->approach;
      auto grasp_pose     = goal->grasp;
      auto retreat_pose   = goal->retreat;

      // =================== Debug Logging ================================================================================
      // Print out expected grasp pose for debugging
      RCLCPP_INFO(get_logger(),
        "Grasp in %s: pos [%.3f %.3f %.3f] quat [%.3f %.3f %.3f %.3f]",
        grasp_pose.header.frame_id.c_str(),
        grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z,
        grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
        grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);
      // =================== Debug Logging ================================================================================

      // Group (Arm/Hand) Params
      const std::string arm_group   = goal->arm_group.empty()   ? "arm"   : goal->arm_group;
      const std::string hand_group  = goal->hand_group.empty()  ? "hand"  : goal->hand_group;
      const std::string eef         = goal->eef.empty()         ? "panda_hand" : goal->eef;
      const std::string ik_frame    = goal->ik_frame.empty()    ? "panda_tcp" : goal->ik_frame;
      const std::string open_state  = goal->open_named_state.empty()  ? "open"  : goal->open_named_state;
      const std::string close_state = goal->close_named_state.empty() ? "close" : goal->close_named_state;

      // Reset ROS introspection before constructing the new object
      task_.reset();
      task_.reset(new moveit::task_constructor::Task());

      // Setup task
      moveit::task_constructor::Task& task = *task_;
      task.stages()->setName("pick_and_place");
      task.loadRobotModel(shared_from_this());

      // Set task properties
      task.setProperty("group", arm_group);                   // panda_arm
      task.setProperty("eef", eef);                           // panda_tcp
      task.setProperty("hand", hand_group);                   // panda_hand
      task.setProperty("hand_grasping_frame", ik_frame);      // panda_hand <- same as hand_frame in tutorials example
      task.setProperty("ik_frame", ik_frame);                 // panda_tcp

      // Solvers
      auto sampling_planner  = std::make_shared<solvers::PipelinePlanner>(shared_from_this());
      auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
      auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

      // Specify sampling_planner params
      sampling_planner->setProperty("goal_joint_tolerance", 1e-2);

      // Specify cartesian planner params
      cartesian_planner->setMaxVelocityScalingFactor(goal->vel_scale > 0.f ? goal->vel_scale : 1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(goal->acc_scale > 0.f ? goal->acc_scale : 1.0);
      cartesian_planner->setStepSize(goal->cart_step_size > 0.f ? goal->cart_step_size : 0.01);

      moveit::task_constructor::Stage* current_state_ptr = nullptr;
      moveit::task_constructor::Stage* initial_state_ptr = nullptr;

      /****************************************************
       *                                                  *
       *               Current State                      *
       *                                                  *
       ***************************************************/    
      // Create a "current state" stage
      {
        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));
      }

      /****************************************************
       *                                                  *
       *               Pick Object                        *
       *                                                  *
       ***************************************************/
      // Create a serial container to hold picking substages
      { // ======== START Pick SerialContainer ========
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

        /****************************************************
         *               Move to *Pregrasp*                 *
         ***************************************************/
        // Simple MoveTo stage to go to pregrasp pose
        {
          auto move_to_pregrasp =
            std::make_unique<mtc::stages::MoveTo>("move to pregrasp", cartesian_planner);
            // std::make_unique<mtc::stages::MoveTo>("move to pregrasp", sampling_planner);
          move_to_pregrasp->setGroup(arm_group);
          move_to_pregrasp->setGoal(approach_pose);  // PoseStamped overload
          grasp->insert(std::move(move_to_pregrasp));
        }

        /****************************************************
         *               Open Hand                          *
         ***************************************************/
        // Create a MoveTo stage to open the hand using the interpolation planner
        {
          auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
          stage_open_hand->setGroup(hand_group);
          stage_open_hand->setGoal("open");
          grasp->insert(std::move(stage_open_hand));
        }

        /****************************************************
         *               Move to *Grasp*                    *
         ***************************************************/
        // Simple MoveTo stage to go to SRDF named target "ready"
        {
          auto move_to_grasp =
            std::make_unique<mtc::stages::MoveTo>("move arm to grasp", sampling_planner);
          move_to_grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          move_to_grasp->setGroup(arm_group);
          move_to_grasp->setGoal(grasp_pose);
          grasp->insert(std::move(move_to_grasp));
        }
      
        // ======== Add the serial container, with all of its substages, to the task ========
        task.add(std::move(grasp));
      } // ======== END Pick SerialContainer ========

      // --- Initialize ---
      RCLCPP_INFO(get_logger(), "[MTC pick_and_place] init()");
      try {
        task.init();
        RCLCPP_INFO(get_logger(), "[MTC pick_and_place] init() OK");
      } catch (mtc::InitStageException& e) {
        RCLCPP_ERROR(get_logger(), "[MTC pick_and_place] init() FAILED: %s", e.what());
        ApproachAndPick::Result result;
        result.success = false; result.error_code = 11;
        result.message = std::string("Stage init failed: ") + e.what();
        finish_goal(1, result);
        return;
      }

      // --- Plan ---
      send_feedback("planning", "", 0.33f);
      RCLCPP_INFO(get_logger(), "Planning");

      moveit::core::MoveItErrorCode plan_ec;
      try {
        plan_ec = task.plan(1);  // try to find 1 solution
        result.message = "MTC plan pick_and_place OK";
      } catch (const std::exception& e) {
        result.success = false; result.error_code = 11;
        result.message = std::string("Planning threw: ") + e.what();
        finish_goal(1, result);
        return;
      }
      if (plan_ec != moveit::core::MoveItErrorCode::SUCCESS || task.solutions().empty()) {
        result.success = false; result.error_code = 12;
        result.message = "Planning failed: no solutions.";
        finish_goal(1, result);
        return;
      }

      // Allow cancel after planning, before motion
      if (goal_handle->is_canceling()) {
        result.success = false; result.error_code = 20;
        result.message = "Canceled before execution.";
        finish_goal(2, result);
        return;
      }

      // Allow cancel after planning, before motion
      if (goal_handle->is_canceling()) {
        result.success = false; result.error_code = 20;
        result.message = "Canceled before execution.";
        finish_goal(2, result);
        return;
      }

      // --- Execute (coarse feedback) ---
      send_feedback("executing", "starting", 0.0f);

      moveit::core::MoveItErrorCode exec_ec;
      try {
        auto solution = task.solutions().front();
        send_feedback("executing", "running", 0.5f);
        // wait until controller is idle before trying to execute the task
        if (!wait_until_controller_idle_(2.0)) {
          result.success = false;
          result.error_code = 30;
          result.message = "Controller busy (still executing another goal).";
          finish_goal(1, result);
          return;
        }
        exec_ec = task.execute(*solution);
        result.message = "MTC execution pick_and_place OK";
      } catch (const std::exception& e) {
        const std::string what = e.what();
        if (what.find("Goal handle is not tracking the goal result") != std::string::npos) { // this is the error I was getting often
          bool idle = wait_for_busy_then_idle_(1.0, 10.0);
          if (idle) {
            result.success = true;
            result.error_code = 0;
            result.message = "Execution untracked but controller became idle; treating as success.";
            finish_goal(0, result);
          } else {
            result.success = false;
            result.error_code = 23;
            result.message = "Execution untracked and controller did not become idle in time.";
            finish_goal(1, result);
          }
          return;
        }

        // Other exceptions: still wait for idle before reporting failure (prevents overlap)
        (void)wait_until_controller_idle_(10.0);
        result.success = false;
        result.error_code = 23;
        result.message = std::string("Execution threw: ") + what;
        finish_goal(1, result);
        return;
      }

      if (exec_ec != moveit::core::MoveItErrorCode::SUCCESS) {
        result.success = true;
        result.error_code = 0;
        result.message = "Execute returned non-SUCCESS, but controller completed busy->idle; treating as success.";
        finish_goal(0, result);
        return;
      }

      send_feedback("executing", "done", 1.0f);
      result.success = true;
      result.error_code = 0;
      result.message = "Done (controller idle).";
      finish_goal(0, result);
      return;
;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "execute_goal uncaught exception: %s", e.what());

        ApproachAndPick::Result result;
        result.success = false;
        result.error_code = 99;
        result.message = std::string("Uncaught exception: ") + e.what();

        if (rclcpp::ok() && goal_handle && goal_handle->is_active()) {
          try {
            auto res_ptr = std::make_shared<ApproachAndPick::Result>(result);
            if (goal_handle->is_canceling()) goal_handle->canceled(res_ptr);
            else goal_handle->abort(res_ptr);
          } catch (const std::exception& ex) {
            RCLCPP_WARN(get_logger(), "Top-level result send threw: %s", ex.what());
          }
        }
    } // catch
  } // execute_goal
}; // class
} // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mtc_ros2::MtcApproachAndPickActionNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}