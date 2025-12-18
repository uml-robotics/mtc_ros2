cmake_minimum_required(VERSION 3.8)
project(mtc_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

# Generate action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MTCPlanAndExecutePick.action"
  "action/MTCApproachAndPick.action"
  "action/MTCRetreatAndPlace.action"
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

# Create executable
add_executable(mtc_plan_and_execute_pick_action src/mtc_plan_and_execute_pick_action.cpp)
ament_target_dependencies(mtc_plan_and_execute_pick_action
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
)

add_executable(mtc_approach_and_pick_action src/mtc_approach_and_pick_action.cpp)
ament_target_dependencies(mtc_approach_and_pick_action
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
)

add_executable(mtc_retreat_and_place_action src/mtc_retreat_and_place_action.cpp)
ament_target_dependencies(mtc_retreat_and_place_action
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_ros_planning_interface
)

install(TARGETS 
  mtc_plan_and_execute_pick_action
  mtc_approach_and_pick_action
  mtc_retreat_and_place_action
  DESTINATION lib/${PROJECT_NAME})

# Link generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mtc_plan_and_execute_pick_action ${cpp_typesupport_target})
target_link_libraries(mtc_approach_and_pick_action ${cpp_typesupport_target})
target_link_libraries(mtc_retreat_and_place_action ${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
