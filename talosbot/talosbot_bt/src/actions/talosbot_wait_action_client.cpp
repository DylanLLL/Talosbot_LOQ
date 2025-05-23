#include "talosbot_bt/actions/talosbot_wait_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{

bool TalosbotWaitActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val = getInput<int>("duration");
  // goal.msec_timeout = timeout.value();
  goal.duration= val.value();
  return true;
}

BT::NodeStatus TalosbotWaitActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: " << wr.result->success << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotWaitActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Feedback received: " << feedback->status << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotWaitActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotWaitActionClient::onHalt()
{
  if (auto node = node_.lock())
  {
    RCLCPP_WARN(node->get_logger(), "TalosbotWaitActionClient: Cancelling goal due to halt");
  }

  RosActionNode::onHalt();
}  // namespace talosbot_bt

}
