#ifndef TALOSBOT_BT__FIBONACCI_ACTION_CLIENT_HPP_
#define TALOSBOT_BT__FIBONACCI_ACTION_CLIENT_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include "action_tutorials_interfaces/action/fibonacci.hpp"

namespace talosbot_bt
{

// To mimic operation of asrs conveyor
// ROS action node with action client depending on external action server
class FibonacciActionClient : public BT::RosActionNode<action_tutorials_interfaces::action::Fibonacci>
{
public:
  FibonacciActionClient(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosActionNode<action_tutorials_interfaces::action::Fibonacci>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>("order") });
    // return {};
  }

  bool setGoal(Goal& goal) override;
  void onHalt() override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__FIBONACCI_ACTION_CLIENT_HPP_
