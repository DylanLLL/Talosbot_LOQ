cmake_minimum_required(VERSION 3.8)
project(talosbot_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(talosbot_custom_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(action_server SHARED
  src/locking_pin_action_server.cpp
  src/go_under_trolley_action_server.cpp
  src/go_middle_front_door_action_server.cpp
  src/move_base_linear_action_server.cpp
  src/wait_action_server.cpp
  src/move_base_angular_action_server.cpp
  src/relocalize_action_server.cpp
  src/template_action_server.cpp
  src/set_tolerance_action_server.cpp)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_compile_definitions(action_server
  PRIVATE "TALOSBOT_ACTION_BUILDING_DLL")

ament_target_dependencies(action_server
  "talosbot_custom_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "nav_msgs"
  )

rclcpp_components_register_node(action_server PLUGIN "talosbot_action::LockingPinActionServer" EXECUTABLE talosbot_locking_pin_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::GoUnderTrolleyActionServer" EXECUTABLE talosbot_go_under_trolley_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::GoMiddleFrontDoorActionServer" EXECUTABLE talosbot_go_middle_front_door_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::MoveBaseLinearActionServer" EXECUTABLE talosbot_move_base_linear_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::WaitActionServer" EXECUTABLE talosbot_wait_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::MoveBaseAngularActionServer" EXECUTABLE talosbot_move_base_angular_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::RelocalizeActionServer" EXECUTABLE talosbot_relocalize_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::TemplateActionServer" EXECUTABLE talosbot_template_action_server)
rclcpp_components_register_node(action_server PLUGIN "talosbot_action::SetToleranceActionServer" EXECUTABLE talosbot_set_tolerance_action_server)


install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


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

ament_package()
