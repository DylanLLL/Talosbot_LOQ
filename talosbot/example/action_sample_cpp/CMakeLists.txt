cmake_minimum_required(VERSION 3.5)
project(action_sample_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_sample_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(action_sample SHARED
  src/fibonacci_action_client.cpp
  src/fibonacci_action_server.cpp
  src/fibonacci_action_nav_client.cpp)
rclcpp_components_register_node(action_sample PLUGIN "action_sample_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
rclcpp_components_register_node(action_sample PLUGIN "action_sample_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
rclcpp_components_register_node(action_sample PLUGIN "action_sample_cpp::FibonacciActionNavClient" EXECUTABLE fibonacci_action_nav_client)

target_compile_definitions(action_sample
  PRIVATE "ACTION_SAMPLE_CPP_BUILDING_DLL")
ament_target_dependencies(action_sample
  "action_sample_interfaces"
  "nav2_msgs"
  "builtin_interfaces"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

install(TARGETS
  action_sample
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
