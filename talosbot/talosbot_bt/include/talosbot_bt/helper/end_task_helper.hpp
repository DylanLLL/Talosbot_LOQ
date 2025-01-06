#ifndef TALOSBOT_BT__PLUGINS__HELPER_END_TASK_HELPER_HPP_
#define TALOSBOT_BT__PLUGINS__HELPER_END_TASK_HELPER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "behaviortree_cpp/behavior_tree.h"
#include "talosbot_bt/httplib.h"

#include "talosbot_bt/nlohmann/json.hpp"

#include <iostream>

namespace talosbot_bt
{
class EndTaskHelper : public BT::SyncActionNode
{
public:
  EndTaskHelper(const std::string& name, const BT::NodeConfig& config) : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("uuid"), BT::InputPort<std::string>("status") };
  }

  BT::NodeStatus tick() override;
};
}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__PLUGINS__HELPER_END_TASK_HELPER_HPP_
