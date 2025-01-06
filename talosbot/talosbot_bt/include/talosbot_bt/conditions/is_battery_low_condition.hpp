#ifndef TALOSBOT_BT__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define TALOSBOT_BT__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "behaviortree_cpp/condition_node.h"
// #include "behaviortree_cpp_v3/condition_node.h"

namespace talosbot_bt
{

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 */
class IsBatteryLowCondition : public BT::ConditionNode, rclcpp::Node
{
public:
  /**
   * @brief A constructor for talosbot_bt::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBatteryLowCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  IsBatteryLowCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
      BT::InputPort<std::string>("battery_topic", std::string("/talosbot1/battery_level"), "Battery topic"),
    };
  }

private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
   */
  void batteryCallback(std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  std::string battery_topic_;
  double min_battery_;
  bool is_battery_low_;
};

}  // namespace talosbot_bt

#endif  // TALOSBOT_BT__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
