#ifndef TALOSBOT_BT__TALOSBOT_LOCKING_PIN_ACTION_CLIENT_HPP_
#define TALOSBOT_BT__TALOSBOT_LOCKING_PIN_ACTION_CLIENT_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include "talosbot_custom_interfaces/action/locking_pin.hpp"

namespace talosbot_bt
{

// To mimic operation of asrs conveyor
// ROS action node with action client depending on external action server
class TalosbotLockingPinActionClient : public BT::RosActionNode<talosbot_custom_interfaces::action::LockingPin>
{
public:
  TalosbotLockingPinActionClient(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosActionNode<talosbot_custom_interfaces::action::LockingPin>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<bool>("up_position") });
    // return {};
  }

  bool setGoal(Goal& goal) override;
  void onHalt() override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__TALOSBOT_LOCKING_PIN_ACTION_CLIENT_HPP_
