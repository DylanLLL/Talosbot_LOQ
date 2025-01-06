#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "talosbot_custom_interfaces/action/wait.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "talosbot_action/visibility_control.h"

namespace talosbot_action
{
class WaitActionServer : public rclcpp::Node
{
public:
  using Wait = talosbot_custom_interfaces::action::Wait;
  using GoalHandleWait = rclcpp_action::ServerGoalHandle<Wait>;

  TALOSBOT_ACTION_PUBLIC

  explicit WaitActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_wait_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Wait>(
        this, "talosbot_wait", std::bind(&WaitActionServer::handle_goal, this, _1, _2),
        std::bind(&WaitActionServer::handle_cancel, this, _1),
        std::bind(&WaitActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Wait>::SharedPtr action_server_;

  void locking_pin_status_callback(const std_msgs::msg::String& msg)
  {
    feedback_ = msg.data;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Wait::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->duration);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&WaitActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWait> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Wait::Feedback>();
    auto result = std::make_shared<Wait::Result>();
    bool goal_achieved = false;
    int count = 0;
    while (!goal_achieved)
    {
      if(count >= goal->duration){
        goal_achieved = true;
        result->success = true;
      }
      count++;
      if (goal_handle->is_canceling())
      {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      feedback->status = feedback_;
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
  std::string feedback_ = "INVALID";
};
}  // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::WaitActionServer)