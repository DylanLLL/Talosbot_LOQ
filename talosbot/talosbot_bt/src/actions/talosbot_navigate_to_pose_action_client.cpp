#include "talosbot_bt/actions/talosbot_navigate_to_pose_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{
bool TalosbotNavigateToPoseActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto target = getInput<std::string>("goal").value();
  // // std::string s = target.value();
  // auto parts = BT::splitString(target, ';');
  // // std::vector<std::string> parts;

  // if (parts.size() != 4)
  // {
  //   throw BT::RuntimeError("invalid input");
  // }
  // else
  // {
  //   double pos_x = std::stod(parts[0]);
  //   double pos_y = std::stod(parts[1]);
  //   double ori_w = std::stod(parts[2]);
  //   double ori_z = std::stod(parts[3]);

  //   goal.pose.header.frame_id = "map";

  //   goal.pose.pose.position.x = pos_x;
  //   goal.pose.pose.position.y = pos_y;
  //   goal.pose.pose.orientation.w = ori_w;
  //   goal.pose.pose.orientation.z = ori_z;
  // }
  
  if (!getInput<double>("goal_x").has_value() || !getInput<double>("goal_y").has_value() ||
      !getInput<double>("quat_w").has_value() || !getInput<double>("quat_z").has_value())
  {
    throw BT::RuntimeError("invalid input");
  }
  goal.pose.header.frame_id = "map";

  goal.pose.pose.position.x = getInput<double>("goal_x").value();
  goal.pose.pose.position.y = getInput<double>("goal_y").value();
  goal.pose.pose.orientation.w = getInput<double>("quat_w").value();
  goal.pose.pose.orientation.z = getInput<double>("quat_z").value();

  return true;
}

BT::NodeStatus TalosbotNavigateToPoseActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  switch (wr.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      std::cout << "Goal was aborted" << std::endl;
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      std::cout << "Goal was canceled" << std::endl;
      return BT::NodeStatus::FAILURE;
    default:
      std::cout << "Unknown result code" << std::endl;
      return BT::NodeStatus::FAILURE;
  }
  std::cout << "Got it done!" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotNavigateToPoseActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Remaining distances (m): " << feedback->distance_remaining << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotNavigateToPoseActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotNavigateToPoseActionClient::onHalt()
{
}
}  // namespace talosbot_bt
