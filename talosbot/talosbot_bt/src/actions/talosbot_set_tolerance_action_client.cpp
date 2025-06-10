#include "talosbot_bt/actions/talosbot_set_tolerance_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{

bool TalosbotSetToleranceActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val1 = getInput<float>("xy_tolerance");
  auto val2 = getInput<float>("yaw_tolerance");
  // goal.msec_timeout = timeout.value();
  goal.xy_tolerance = val1.value();
  goal.yaw_tolerance = val2.value();
  return true;
}

BT::NodeStatus TalosbotSetToleranceActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: " << wr.result->success << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotSetToleranceActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Feedback received: " << feedback->status << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotSetToleranceActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotSetToleranceActionClient::onHalt()
{
  if (auto node = node_.lock())
  {
    RCLCPP_WARN(node->get_logger(), "TalosbotSetToleranceActionClient: Cancelling goal due to halt");
  }

  RosActionNode::onHalt();
}
}  // namespace talosbot_bt
