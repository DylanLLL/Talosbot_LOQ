#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/bt_sqlite_logger.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <talosbot_bt/actions/fibonacci_action_client.hpp>
#include <talosbot_bt/actions/talosbot_locking_pin_action_client.hpp>
#include <talosbot_bt/actions/talosbot_navigate_to_pose_action_client.hpp>
#include <talosbot_bt/actions/talosbot_go_under_trolley_action_client.hpp>
#include <talosbot_bt/actions/talosbot_go_middle_front_door_action_client.hpp>
#include <talosbot_bt/actions/talosbot_move_base_linear_action_client.hpp>
#include <talosbot_bt/actions/talosbot_wait_action_client.hpp>
#include <talosbot_bt/actions/talosbot_move_base_angular_action_client.hpp>
#include <talosbot_bt/actions/talosbot_relocalize_action_client.hpp> 


#include <talosbot_bt/conditions/is_battery_low_condition.hpp>

#include <talosbot_bt/helper/get_next_pose_helper.hpp>
#include <talosbot_bt/helper/get_pose_helper.hpp>
#include <talosbot_bt/helper/print_pose_helper.hpp>
#include <talosbot_bt/helper/end_task_helper.hpp>

#include <string>

using FibonacciActionClient = talosbot_bt::FibonacciActionClient;
using TalosbotLockingPinActionClient = talosbot_bt::TalosbotLockingPinActionClient;
using TalosbotNavigateToPoseActionClient = talosbot_bt::TalosbotNavigateToPoseActionClient;
using TalosbotGoUnderTrolleyActionClient = talosbot_bt::TalosbotGoUnderTrolleyActionClient;
using TalosbotGoMiddleFrontDoorActionClient = talosbot_bt::TalosbotGoMiddleFrontDoorActionClient;
using TalosbotMoveBaseLinearActionClient = talosbot_bt::TalosbotMoveBaseLinearActionClient;
using TalosbotWaitActionClient = talosbot_bt::TalosbotWaitActionClient;
using TalosbotMoveBaseAngularActionClient = talosbot_bt::TalosbotMoveBaseAngularActionClient;
using TalosbotRelocalizeActionClient = talosbot_bt::TalosbotRelocalizeActionClient;


using IsBatteryLowCondition = talosbot_bt::IsBatteryLowCondition;

using GetNextPoseHelper = talosbot_bt::GetNextPoseHelper;
using GetPoseHelper = talosbot_bt::GetPoseHelper;
using PrintPoseHelper = talosbot_bt::PrintPoseHelper;
using EndTaskHelper = talosbot_bt::EndTaskHelper;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("talosbot_bt_node");

  // Get Params
  std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("talosbot_bt") + "/behavior_trees/trial_tree.xml";

  nh->declare_parameter("bt_file", pkg_share_dir);
  nh->declare_parameter("log_path", "./");

  std::string bt_file_;
  nh->get_parameter_or<std::string>("bt_file", bt_file_, pkg_share_dir);

  std::string log_path_;
  nh->get_parameter_or<std::string>("log_path", log_path_, "./");

  BT::RosNodeParams params;
  params.nh = nh;

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<FibonacciActionClient>("Fibonacci", params);
  factory.registerNodeType<TalosbotLockingPinActionClient>("TalosbotLockingPin", params);
  factory.registerNodeType<TalosbotNavigateToPoseActionClient>("TalosbotNavigateToPose", params);
  factory.registerNodeType<TalosbotGoUnderTrolleyActionClient>("TalosbotGoUnderTrolley", params);
  factory.registerNodeType<TalosbotGoMiddleFrontDoorActionClient>("TalosbotGoMiddleFrontDoor", params);
  factory.registerNodeType<TalosbotMoveBaseLinearActionClient>("TalosbotMoveBaseLinear", params);
  factory.registerNodeType<TalosbotWaitActionClient>("TalosbotWait", params);
  factory.registerNodeType<TalosbotMoveBaseAngularActionClient>("TalosbotMoveBaseAngular", params);
  factory.registerNodeType<TalosbotRelocalizeActionClient>("TalosbotRelocalize", params); 
 

  factory.registerNodeType<IsBatteryLowCondition>("IsBatteryLow");

  factory.registerNodeType<GetNextPoseHelper>("GetNextPoseHelper");
  factory.registerNodeType<GetPoseHelper>("GetPoseHelper");
  factory.registerNodeType<PrintPoseHelper>("PrintPoseHelper");
  factory.registerNodeType<EndTaskHelper>("EndTaskHelper");

  auto tree = factory.createTreeFromFile(bt_file_);

  BT::StdCoutLogger cout_logger(tree);
  BT::Groot2Publisher publisher(tree);

  std::string btlog_file_ = log_path_ + "generated_log.btlog";
  std::string sqlite_file_ = log_path_ + "generated_log.db3";

  BT::FileLogger2 file_logger(tree, btlog_file_);
  BT::SqliteLogger sqlite_logger(tree, sqlite_file_);
  
  BT::printTreeRecursively(tree.rootNode());

  tree.tickWhileRunning();

  return 0;
}
