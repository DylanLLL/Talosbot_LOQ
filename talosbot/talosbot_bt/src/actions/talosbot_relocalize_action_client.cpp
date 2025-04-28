#include "talosbot_bt/actions/talosbot_relocalize_action_client.hpp"
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp/rclcpp.hpp>

namespace talosbot_bt
{

bool TalosbotRelocalizeActionClient::setGoal(RosActionNode::Goal& goal)
{
    // auto timeout = getInput<unsigned>("msec");
    auto val = getInput<bool>("relocalize");
    // goal.msec_timeout = timeout.value();
    goal.relocalize = val.value();
    return true;
}

BT::NodeStatus TalosbotRelocalizeActionClient::onResultReceived(const RosActionNode::WrappedResult& wr)
{
    std::cout << "Result received: " << wr.result->success << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TalosbotRelocalizeActionClient::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    std::cout << "Feedback received: " << feedback->status << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TalosbotRelocalizeActionClient::onFailure(BT::ActionNodeErrorCode error)
{
    std::cout << error << std::endl;
    return BT::NodeStatus::FAILURE;
}

void TalosbotRelocalizeActionClient::onHalt()
{
    if (auto node = node_.lock())
  {
    RCLCPP_WARN(node->get_logger(), "TalosbotRelocalizeActionClient: Cancelling goal due to halt");
  }

  RosActionNode::onHalt();
}

}  // namespace talosbot_bt
