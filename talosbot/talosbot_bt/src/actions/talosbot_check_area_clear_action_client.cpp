#include "talosbot_bt/actions/talosbot_check_area_clear_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{
bool TalosbotCheckAreaClearActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val = getInput<bool>("check");
  // goal.msec_timeout = timeout.value();
  goal.check_area = val.value();
  return true;
}

BT::NodeStatus TalosbotCheckAreaClearActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: " << wr.result->is_clear << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotCheckAreaClearActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Feedback received: " << feedback->status << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotCheckAreaClearActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotCheckAreaClearActionClient::onHalt()
{
  if (auto node = node_.lock())
  {
    RCLCPP_WARN(node->get_logger(), "TalosbotCheckAreaClearActionClient: Cancelling goal due to halt");
  }

  RosActionNode::onHalt();
}
}  // namespace talosbot_bt
