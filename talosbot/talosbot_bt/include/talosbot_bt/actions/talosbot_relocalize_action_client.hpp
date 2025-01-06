#ifndef TALOSBOT_BT__TALOSBOT_RELOCALIZE_ACTION_CLIENT_HPP_
#define TALOSBOT_BT__TALOSBOT_RELOCALIZE_ACTION_CLIENT_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "talosbot_custom_interfaces/action/relocalize.hpp"

namespace talosbot_bt
{

class TalosbotRelocalizeActionClient : public BT::RosActionNode <talosbot_custom_interfaces::action::Relocalize>
{
public:
    TalosbotRelocalizeActionClient(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosActionNode<talosbot_custom_interfaces::action::Relocalize>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({ BT::InputPort<bool>("relocalize") });
        // return {};
    }

    bool setGoal(RosActionNode::Goal& goal) override;

    BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult& wr) override;

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

    void onHalt() override;
};
}  // namespace talosbot_bt

#endif  // RELOCALIZE_ACTION_CLIENT_HPP