#include <string>

#include "talosbot_bt/conditions/is_battery_low_condition.hpp"

namespace talosbot_bt
{

IsBatteryLowCondition::IsBatteryLowCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
  : BT::ConditionNode(condition_name, conf)
  , Node("is_battery_low_node")
  , battery_topic_("/battery_status")
  , min_battery_(0.0)
  , is_battery_low_(false)
{
  getInput("min_battery", min_battery_);
  getInput("battery_topic", battery_topic_);
  // node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("talosbot_bt_node");
  node_ = rclcpp::Node::make_shared("is_battery_low_condition_node");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  battery_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      battery_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1), sub_option);
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_battery_low_)
  {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(std_msgs::msg::Float32::SharedPtr msg)
{
  is_battery_low_ = msg->data <= min_battery_;
}

}  // namespace talosbot_bt

// #include "behaviortree_cpp/bt_factory.h"
// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<talosbot_bt::IsBatteryLowCondition>("IsBatteryLow");
// }
