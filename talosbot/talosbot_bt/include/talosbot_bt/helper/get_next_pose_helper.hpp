#ifndef TALOSBOT_BT__PLUGINS__HELPER_GET_NEXT_POSE_HELPER_HPP_
#define TALOSBOT_BT__PLUGINS__HELPER_GET_NEXT_POSE_HELPER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "talosbot_bt/httplib.h"

#include "talosbot_bt/nlohmann/json.hpp"

#include <iostream>

namespace talosbot_bt
{
class GetNextPoseHelper : public BT::SyncActionNode
{
public:
  GetNextPoseHelper(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<double>("pos_x"),           BT::OutputPort<double>("pos_y"),
             BT::OutputPort<double>("quat_z"),          BT::OutputPort<double>("quat_w"),
             BT::OutputPort<std::string>("uuid"),       BT::OutputPort<std::string>("locations"),
             BT::OutputPort<std::string>("post_action") };
  }

  BT::NodeStatus tick() override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__PLUGINS__HELPER_GET_NEXT_POSE_HELPER_HPP_
