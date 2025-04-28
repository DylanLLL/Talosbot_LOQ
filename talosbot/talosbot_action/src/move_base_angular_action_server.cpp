#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <math.h>
#include <iostream>

#include "talosbot_custom_interfaces/action/move_base_angular.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include "talosbot_action/visibility_control.h"

namespace talosbot_action
{
class MoveBaseAngularActionServer : public rclcpp::Node
{
public:
  using MoveBaseAngular = talosbot_custom_interfaces::action::MoveBaseAngular;
  using GoalHandleMoveBaseAngular = rclcpp_action::ServerGoalHandle<MoveBaseAngular>;

  TALOSBOT_ACTION_PUBLIC

  explicit MoveBaseAngularActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_move_base_angular_action_server", options)
  {
    using namespace std::placeholders;
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    this->action_server_ = rclcpp_action::create_server<MoveBaseAngular>(
        this, "talosbot_move_base_angular", 
        std::bind(&MoveBaseAngularActionServer::handle_goal, this, _1, _2),
        std::bind(&MoveBaseAngularActionServer::handle_cancel, this, _1),
        std::bind(&MoveBaseAngularActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MoveBaseAngular>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const MoveBaseAngular::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->angle);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveBaseAngular> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveBaseAngular> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&MoveBaseAngularActionServer::execute, this, _1), goal_handle }.detach();
  }

  float signChecker(float number){
    if (number > 0){
      return 1;
    }
    else{
      return -1;
    }
  }

  void execute(const std::shared_ptr<GoalHandleMoveBaseAngular> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveBaseAngular::Feedback>();
    auto result = std::make_shared<MoveBaseAngular::Result>();
    bool goal_achieved = false;
    int counter = 0;
    float rotate_speed = 0.2;
    float talosbot_length = 1.2;

    while (!goal_achieved)
    {
      float target = (fabs(goal->angle)) * (M_PI/180) * talosbot_length;

      if(counter >= 10 * (target /rotate_speed)){
        geometry_msgs::msg::Twist vel_msg_;
        vel_msg_.linear.x = 0.00;
        vel_msg_.linear.y = 0.00;
        vel_msg_.angular.z = 0.00;
        cmd_vel_pub_->publish(vel_msg_);
        goal_achieved=true;
        result->success = true;
      }

      else{
        geometry_msgs::msg::Twist vel_msg_;
        vel_msg_.linear.x = 0.00;
        vel_msg_.linear.y = 0.00;
        vel_msg_.angular.z = signChecker(goal->angle) * rotate_speed;
        cmd_vel_pub_->publish(vel_msg_);
        counter++;
      }

      if (goal_handle->is_canceling())
      {
        geometry_msgs::msg::Twist vel_msg_;
        vel_msg_.linear.x = 0.00;
        vel_msg_.linear.y = 0.00;
        vel_msg_.angular.z = 0.00;
        cmd_vel_pub_->publish(vel_msg_);

        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->status = std::to_string((target/rotate_speed) * 10);
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

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::MoveBaseAngularActionServer)