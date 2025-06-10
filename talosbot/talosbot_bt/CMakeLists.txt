cmake_minimum_required(VERSION 3.8)
project(talosbot_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

find_package(action_tutorials_interfaces REQUIRED)
find_package(talosbot_custom_interfaces REQUIRED)


include_directories(include)

add_executable(
  start_tree
  src/main.cpp
  src/actions/fibonacci_action_client.cpp
  src/actions/talosbot_locking_pin_action_client.cpp
  src/actions/talosbot_navigate_to_pose_action_client.cpp
  src/actions/talosbot_go_under_trolley_action_client.cpp
  src/actions/talosbot_go_middle_front_door_action_client.cpp
  src/actions/talosbot_move_base_linear_action_client.cpp
  src/actions/talosbot_move_base_angular_action_client.cpp
  src/actions/talosbot_wait_action_client.cpp
  src/actions/talosbot_relocalize_action_client.cpp
  src/actions/talosbot_set_tolerance_action_client.cpp
  src/conditions/is_battery_low_condition.cpp
  src/helper/get_next_pose_helper.cpp
  src/helper/get_pose_helper.cpp
  src/helper/print_pose_helper.cpp
  src/helper/end_task_helper.cpp
)

ament_target_dependencies(
  start_tree
  behaviortree_cpp
  behaviortree_ros2
  std_msgs
  geometry_msgs
  nav2_msgs
  action_tutorials_interfaces
  talosbot_custom_interfaces
  btcpp_ros2_interfaces
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

install(TARGETS
  start_tree
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and licenseaction_tutorials_interfaces is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
