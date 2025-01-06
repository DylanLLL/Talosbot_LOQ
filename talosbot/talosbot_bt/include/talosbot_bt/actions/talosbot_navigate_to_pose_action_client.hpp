#ifndef TALOSBOT_BT__TALOSBOT_NAVIGATE_TO_POSE_ACTION_CLIENT_HPP_
#define TALOSBOT_BT__TALOSBOT_NAVIGATE_TO_POSE_ACTION_CLIENT_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <sstream>
#include <vector>

namespace talosbot_bt
{

// To mimic operation of asrs conveyor
// ROS action node with action client depending on external action server
class TalosbotNavigateToPoseActionClient : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  TalosbotNavigateToPoseActionClient(const std::string& name, const BT::NodeConfig& conf,
                                     const BT::RosNodeParams& params)
    : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<double>("goal_x"),
        BT::InputPort<double>("goal_y"),
        BT::InputPort<double>("quat_w"),
        BT::InputPort<double>("quat_z"),
    });
    // return {};
  }

  bool setGoal(Goal& goal) override;
  void onHalt() override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__TALOSBOT_NAVIGATE_TO_POSE_ACTION_CLIENT_HPP_
