#include "talosbot_bt/actions/talosbot_locking_pin_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{
bool TalosbotLockingPinActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val = getInput<bool>("up_position");
  // goal.msec_timeout = timeout.value();
  goal.up_position = val.value();
  return true;
}

BT::NodeStatus TalosbotLockingPinActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: " << wr.result->success << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotLockingPinActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Feedback received: " << feedback->status << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotLockingPinActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotLockingPinActionClient::onHalt()
{
  if (auto node = node_.lock())
  {
    RCLCPP_WARN(node->get_logger(), "TalosbotLockingPinActionClient: Cancelling goal due to halt");
  }

  RosActionNode::onHalt();
}
}  // namespace talosbot_bt
