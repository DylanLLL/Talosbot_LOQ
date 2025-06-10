#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "talosbot_custom_interfaces/action/set_tolerance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "talosbot_action/visibility_control.h"

namespace talosbot_action
{
class SetToleranceActionServer : public rclcpp::Node
{
public:
  using SetTolerance = talosbot_custom_interfaces::action::SetTolerance;
  using GoalHandleSetTolerance = rclcpp_action::ServerGoalHandle<SetTolerance>;

  TALOSBOT_ACTION_PUBLIC

  explicit SetToleranceActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_set_tolerance_action_server", options)
  {
    using namespace std::placeholders;

    // locking_pin_command_pub_ = this->create_publisher<std_msgs::msg::Bool>("talosbot1/locking_pin_command", 10);
    // local_costmap_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("local_costmap/footprint", 10);
    // global_costmap_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("global_costmap/footprint", 10);


    // locking_pin_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    //     "talosbot1/locking_pin_status", 10,
    //     std::bind(&SetToleranceActionServer::set_tolerance_status_callback, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<SetTolerance>(
        this, "talosbot_set_tolerance", std::bind(&SetToleranceActionServer::handle_goal, this, _1, _2),
        std::bind(&SetToleranceActionServer::handle_cancel, this, _1),
        std::bind(&SetToleranceActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<SetTolerance>::SharedPtr action_server_;

  void set_tolerance_status_callback(const std_msgs::msg::String& msg)
  {
    feedback_ = msg.data;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const SetTolerance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->xy_tolerance);
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->yaw_tolerance);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetTolerance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetTolerance> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&SetToleranceActionServer::execute, this, _1), goal_handle }.detach();
  }

  void update_nav2_parameters(double xy_goal_tolerance_value, double yaw_goal_tolerance_value){
    // Adjust the Nav2 xy_goal_tolerance parameter dynamically
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "controller_server");
    if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to parameter server!");
        return;
    }

  // Create the parameter update asynchronously
    parameters_client->set_parameters_atomically({rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", xy_goal_tolerance_value)});
    parameters_client->set_parameters_atomically({rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", yaw_goal_tolerance_value)});
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    RCLCPP_INFO(this->get_logger(), "xy_goal_tolerance set to: %f", xy_goal_tolerance_value);
    RCLCPP_INFO(this->get_logger(), "yaw_goal_tolerance set to: %f", yaw_goal_tolerance_value);
  }

  void execute(const std::shared_ptr<GoalHandleSetTolerance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetTolerance::Feedback>();
    auto result = std::make_shared<SetTolerance::Result>();
    bool goal_achieved = false;

    while (!goal_achieved)
    {
        for (int i = 0; i < 2; ++i)
        {
            update_nav2_parameters(goal->xy_tolerance, goal->yaw_tolerance);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        result->success = true;
        goal_achieved = true;

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
  // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr locking_pin_command_pub_;
  // rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_costmap_pub_;
  // rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_costmap_pub_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr locking_pin_status_sub_;
  std::string feedback_ = "INVALID";
};
}  // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::SetToleranceActionServer)