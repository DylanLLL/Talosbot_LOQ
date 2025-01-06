#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "talosbot_custom_interfaces/action/move_base_linear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include "talosbot_action/visibility_control.h"

namespace talosbot_action
{
class MoveBaseLinearActionServer : public rclcpp::Node
{
public:
  using  MoveBaseLinear = talosbot_custom_interfaces::action::MoveBaseLinear;
  using GoalHandleMoveBaseLinear = rclcpp_action::ServerGoalHandle<MoveBaseLinear>;

  TALOSBOT_ACTION_PUBLIC

  explicit MoveBaseLinearActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_move_base_linear_action_server", options)
  {
    using namespace std::placeholders;
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    this->action_server_ = rclcpp_action::create_server<MoveBaseLinear>(
        this, "talosbot_move_base_linear", std::bind(&MoveBaseLinearActionServer::handle_goal, this, _1, _2),
        std::bind(&MoveBaseLinearActionServer::handle_cancel, this, _1),
        std::bind(&MoveBaseLinearActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MoveBaseLinear>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const MoveBaseLinear::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->dist);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveBaseLinear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveBaseLinear> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&MoveBaseLinearActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveBaseLinear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveBaseLinear::Feedback>();
    auto result = std::make_shared<MoveBaseLinear::Result>();
    bool goal_achieved = false;
    int counter = 0;
    while (!goal_achieved)
    {
      float target = fabs(goal->dist);
      if(counter >= 10 * (target /0.2)){
        geometry_msgs::msg::Twist vel_msg_;
        vel_msg_.linear.x = 0.00;
        vel_msg_.linear.y = 0.00;
        vel_msg_.angular.z = 0.00;
        cmd_vel_pub_->publish(vel_msg_);
        goal_achieved=true;
        result->success = true;
        // return;
      }else{
        geometry_msgs::msg::Twist vel_msg_;
        vel_msg_.linear.x = ((goal->dist) / fabs(goal->dist)) * 0.2;
        vel_msg_.linear.y = 0.00;
        vel_msg_.angular.z = 0.00;
        cmd_vel_pub_->publish(vel_msg_);
        counter++;
      }
      if (goal_handle->is_canceling())
      {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->status = std::to_string((target/0.2) * 10);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
    if (rclcpp::ok())
    {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  std::string feedback_ = "INVALID";
};
}  // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::MoveBaseLinearActionServer)