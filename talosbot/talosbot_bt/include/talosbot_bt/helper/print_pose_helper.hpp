#ifndef TALOSBOT_BT__PLUGINS__HELPER_PRINT_POSE_HELPER_HPP_
#define TALOSBOT_BT__PLUGINS__HELPER_PRINT_POSE_HELPER_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include <cstdio>

namespace talosbot_bt
{
class PrintPoseHelper : public BT::SyncActionNode
{
public:
  PrintPoseHelper(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pos_x"),
      BT::InputPort<double>("pos_y"),
      BT::InputPort<double>("quat_z"),
      BT::InputPort<double>("quat_w"),
    };
  }

  BT::NodeStatus tick() override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__PLUGINS__HELPER_PRINT_POSE_HELPER_HPP_
