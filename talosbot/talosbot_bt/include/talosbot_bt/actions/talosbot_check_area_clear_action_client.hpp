#ifndef TALOSBOT_BT__TALOSBOT_CHECK_AREA_CLEAR_HPP_
#define TALOSBOT_BT__TALOSBOT_CHECK_AREA_CLEAR_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include "talosbot_custom_interfaces/action/check_area_clear.hpp"

namespace talosbot_bt
{

// To detect if area is clear
// ROS action node with action client depending on external action server
class TalosbotCheckAreaClearActionClient : public BT::RosActionNode<talosbot_custom_interfaces::action::CheckAreaClear>
{
public:
  TalosbotCheckAreaClearActionClient(const std::string& name, const BT::NodeConfig& conf,
                                     const BT::RosNodeParams& params)
    : RosActionNode<talosbot_custom_interfaces::action::CheckAreaClear>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<bool>("check") });
    // return {};
  }

  bool setGoal(Goal& goal) override;
  void onHalt() override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__TALOSBOT_CHECK_AREA_CLEAR_ACTION_CLIENT_HPP_
