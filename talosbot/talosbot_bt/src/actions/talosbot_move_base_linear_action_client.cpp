#include "talosbot_bt/actions/talosbot_move_base_linear_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{
bool TalosbotMoveBaseLinearActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val = getInput<int>("dist");
  // goal.msec_timeout = timeout.value();
  goal.dist = val.value();
  return true;
}

BT::NodeStatus TalosbotMoveBaseLinearActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: " << wr.result->success << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotMoveBaseLinearActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Feedback received: " << feedback->status << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotMoveBaseLinearActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void TalosbotMoveBaseLinearActionClient::onHalt()
{
}
}  // namespace talosbot_bt
