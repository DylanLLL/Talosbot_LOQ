#include "talosbot_bt/actions/fibonacci_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{
bool FibonacciActionClient::setGoal(RosActionNode::Goal& goal)
{
  // auto timeout = getInput<unsigned>("msec");
  auto val = getInput<int>("order");
  // goal.msec_timeout = timeout.value();
  goal.order = val.value();
  return true;
}

BT::NodeStatus FibonacciActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  std::cout << "Result received: ";
  for (auto number : wr.result->sequence)
  {
    std::cout << number << " ";
  }
  // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
  std::cout << std::endl;
  // return NodeStatus::SUCCESS;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FibonacciActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::cout << "Next number in sequence received: ";
  for (auto number : feedback->partial_sequence)
  {
    std::cout << number << " ";
  }
  // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
  std::cout << std::endl;
  // return NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FibonacciActionClient::onFailure(BT::ActionNodeErrorCode error)
{
  std::cout << error << std::endl;
  return BT::NodeStatus::FAILURE;
}

void FibonacciActionClient::onHalt()
{
}
}  // namespace talosbot_bt
