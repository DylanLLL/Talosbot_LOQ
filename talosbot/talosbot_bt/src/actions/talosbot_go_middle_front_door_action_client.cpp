#include "talosbot_bt/actions/talosbot_go_middle_front_door_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{
bool TalosbotGoMiddleFrontDoorActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val = getInput<bool>("go");
  // goal.msec_timeout = timeout.value();
  goal.go_middle_front_door = val.value();
  return true;
}

BT::NodeStatus TalosbotGoMiddleFrontDoorActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: " << wr.result->success << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotGoMiddleFrontDoorActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Feedback received: " << feedback->status << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotGoMiddleFrontDoorActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotGoMiddleFrontDoorActionClient::onHalt()
{
  if (auto node = node_.lock())
  {
    RCLCPP_WARN(node->get_logger(), "TalosbotGoMiddleFrontDoorActionClient: Cancelling goal due to halt");
  }

  RosActionNode::onHalt();
}
}  // namespace talosbot_bt
