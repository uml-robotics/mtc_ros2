# MTC_ROS2

This package provides ROS 2 **service servers and/or action servers** that wrap lower-level utilities
(e.g., PCL, perception, planning, or motion helpers) into reusable ROS 2 nodes.

These nodes are intended to be used as modular building blocks in larger systems such as
state machines, behavior trees, or task planners.

## ROS 2 Version

This package is built for **ROS 2** and has been **tested on ROS 2 Jazzy**.

## Package Overview

- Implements one or more **ROS 2 service servers and/or action servers**
- Designed to be called by higher-level coordination logic (FlexBE, SMACH, Behavior Trees, etc.)
- Focuses on wrapping non-ROS utilities into clean ROS 2 interfaces
- Intended for composition into larger manipulation or perception pipelines

## Requirements

This package depends on the following:

### System Dependencies
- Ubuntu 24.04 (tested)
- ROS 2 Jazzy

### ROS / Library Dependencies
```text
- rclcpp
- geometry_msgs
- moveit_ros_planning_interface
- moveit_core
- moveit_task_constructor_core
- moveit_task_constructor_msgs
```

## Mandatory Conditional Dependency:
>Users **must** follow the instructions at the
>[MoveIt Tutorials Setup Instructions](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
>page to **install MoveIt from source** befure using the **MoveIt Task Constructor** in ROS2 Jazzy.
>
>Make sure all dependencies are available in the ROS 2 workspace by following these instructions and then sourcing your new MoveIt workspace before building this >package and any other packages which rely on MoveIt or any of its libraries.

## Build

>In order to utilize the MoveIt Task Constructor (MTC) functionality in ROS2 Jazzy, you will need to have completed the dependency requirements above and source the workspace you created when installing the MoveIt libraries and packages from source.
>
>If you followed the guide directly, that will be ~/ws_moveit, however you can use your own workspace name if desired and then make modifications to the instructions below:

From the root of your ROS 2 workspace:

```bash
source ~/ws_moveit/install/setup.bash
colcon build --packages-select mtc_ros2
source install/setup.bash
```

## Usage

The nodes can be run directly, but rely on parameters such as the robot_description which are passed to the node and should be implemented into a launch file as demonstrated below:


**example.launch.py**
```python
def launch_setup(context, *args, **kwargs):
  robot_description = ...
  ...

  # example launch file implementation for mtc_plan_and_execute_pick_action node
  mtc_plan_and_execute_pick = Node(
        package="mtc_flexbe_utilities",
        executable="mtc_plan_and_execute_pick_action",
        name="mtc_plan_and_execute_pick_action",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            joint_limits_yaml,
            ompl_planning_pipeline_config_mtc,
        ],
    )

  return [
      mtc_plan_and_execute_pick_action,
  ]
```

>A full example launch file implementation can be found at [gazebo_move_group_flexbe.launch.py](https://github.com/uml-robotics/armada_ros2/blob/main/armada_bringup/launch/gazebo_move_group_flexbe.launch.py)

## Interfaces

>In their current state, *approach_and_pick* as well as *retreat_and_place* were experimental and **plan_and_execute_pick** is the successful implementation of the full routine. 
>
>The secondary/experimental nodes will be replaced or simplified in a future update.

### Primary Action

| Action Name | Type | Description |
|-------------|------|-------------|
| `/mtc_plan_and_execute_pick_action` | `action/MTCApproachAndPick` | Given a *geometry_msgs/PoseStamped* **approach**, *geometry_msgs/PoseStamped* **grasp**, and *geometry_msgs/PoseStamped* **retreat**,  approach the target location, close the gripper, lift and retreat with the object, move to a dropoff location and open the gripper|

### Secondary/Experimental Actions

| Action Name | Type | Description |
|-------------|------|-------------|
| `/mtc_approach_and_pick_action` | `action/MTCApproachAndPick` | Given a *geometry_msgs/PoseStamped* **approach**, *geometry_msgs/PoseStamped* **grasp**, and *geometry_msgs/PoseStamped* **retreat**,  approach the target location and close the gripper|
| `/mtc_approach_and_pick_action` | `action/MTCApproachAndPick` | Given a *geometry_msgs/PoseStamped* **approach**, *geometry_msgs/PoseStamped* **grasp**, and *geometry_msgs/PoseStamped* **retreat**,  lift the gripper and retreat to a dropoff position and open the gripper|

## Implementation

Below is an example implementation of a MoveIt Task Constructor (MTC) node to demonstrate functionality. A full tutorial of the MTC can be found at the [MoveIt Task Constructor Documentation Page](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html) where visuals and tutorials can be found and explored.

>Even this stripped-down example code is admittedly heavy, it is **strongly recommended** that the user follow through the [MTC Tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) on the MTC documentation pages before attempting to make their own MTC nodes.

```cpp
#include ...

// use some namespaces for convenience
namespace mtc = moveit::task_constructor;
namespace stages = mtc::stages;
namespace solvers = mtc::solvers;

namespace mtc_ros2 {

using ApproachAndPick = mtc_ros2::action::MTCApproachAndPick;
using GoalHandle     = rclcpp_action::ServerGoalHandle<ApproachAndPick>;

// create a class for our action server
class MtcApproachAndPickActionNode : public rclcpp::Node {
public:
  explicit MtcApproachAndPickActionNode()
  : rclcpp::Node("mtc_approach_and_pick_node")
  {
    // initialize action server in constructor
    action_server_ = rclcpp_action::create_server<ApproachAndPick>(
      this,
      "mtc_approach_and_pick",
      std::bind(&MtcApproachAndPickActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MtcApproachAndPickActionNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MtcApproachAndPickActionNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "MTC Approach and Pick action server ready.");
  }

private:
// create shared variables
  moveit::task_constructor::TaskPtr task_;
  rclcpp_action::Server<ApproachAndPick>::SharedPtr action_server_;

  // set up required action components handle_goal, handle_cancel, handle_accepted
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

  // implement actual functionality in execute_goal function
  void execute_goal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    ApproachAndPick::Result result;

    auto send_feedback = [&](const std::string& phase,
                             const std::string& stage, float prog){
      auto fb = std::make_shared<ApproachAndPick::Feedback>();
      fb->phase = phase;
      fb->stage_name = stage;
      fb->stage_progress = prog;
      goal_handle->publish_feedback(fb);
    };

    // BEFORE constructing stages, set grasp pose to provided target grasp pose:
    auto grasp_pose     = goal->grasp;

    // Group (Arm/Hand) Params currently specified for the Panda robot
    const std::string arm_group   = goal->arm_group.empty()   ? "panda_arm"   : goal->arm_group;
    const std::string hand_group  = goal->hand_group.empty()  ? "panda_hand"  : goal->hand_group;
    const std::string eef         = goal->eef.empty()         ? "panda_tcp" : goal->eef;
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
    task.setProperty("group", arm_group);
    task.setProperty("eef", eef);
    task.setProperty("hand", hand_group);
	  task.setProperty("hand_grasping_frame", hand_group);
    task.setProperty("ik_frame", ik_frame);

    // Solvers
    auto sampling_planner  = std::make_shared<solvers::PipelinePlanner>(shared_from_this());
    auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
    auto interpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();

    // Specify sampling_planner (OMPL) params
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

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
     *          Pick Object Serial Container            *
     *                                                  *
     ***************************************************/
    // Create a serial container to hold picking substages
    { // ======== START Pick SerialContainer ========
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
      grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

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
          std::make_unique<mtc::stages::MoveTo>("move arm to ready", sampling_planner);
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
      std::stringstream ss;
      ss << "Init failed in debug_generate_pose:\n" << e;
      RCLCPP_ERROR_STREAM(get_logger(), ss.str());
      ApproachAndPick::Result result;
      result.success = false;
      result.message = std::string("Stage init failed: ") + e.what();
      goal_handle->abort(std::make_shared<ApproachAndPick::Result>(result));
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
      result.success = false;
      result.message = std::string("Planning threw: ") + e.what();
      goal_handle->abort(std::make_shared<ApproachAndPick::Result>(result));
      return;
    }
    if (plan_ec != moveit::core::MoveItErrorCode::SUCCESS || task.solutions().empty()) {
      result.success = false;
      result.message = "Planning failed: no solutions.";
      goal_handle->abort(std::make_shared<ApproachAndPick::Result>(result));
      return;
    }

    // Allow cancel after planning, before motion
    if (goal_handle->is_canceling()) {
      result.success = false;
      result.message = "Canceled before execution.";
      goal_handle->canceled(std::make_shared<ApproachAndPick::Result>(result));
      return;
    }

    // --- Execute (coarse feedback) ---
    send_feedback("executing", "starting", 0.0f);

    moveit::core::MoveItErrorCode exec_ec;
    try {
      auto solution = task.solutions().front();
      send_feedback("executing", "running", 0.5f);
      exec_ec = task.execute(*solution);
      result.message = "MTC execution pick_and_place OK";
    } catch (const std::exception& e) {
      result.success = true;
      result.message = std::string("Execution threw (soft success): ") + e.what();
      goal_handle->succeed(std::make_shared<ApproachAndPick::Result>(result));
      return;
    }

    if (exec_ec != moveit::core::MoveItErrorCode::SUCCESS) {
      result.success = false;
      result.message = "Execution failed.";
      goal_handle->abort(std::make_shared<ApproachAndPick::Result>(result));
      return;
    }

    send_feedback("executing", "done", 1.0f);
    result.success = true; 
    result.message = "Approach and Pick succeeded.";
    goal_handle->succeed(std::make_shared<ApproachAndPick::Result>(result));

  RCLCPP_INFO(get_logger(), "Something else went wrong - Returning");
  return;
};
};
}

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
```

> This is a slightly stripped-down version of the mtc_approach_and_pick_action node which exists in this package under src/, please check out that node for more information. Once again it is **strongly recommended** that the user follow through the [MTC Tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) on the MTC documentation pages before attempting to make their own MTC nodes.