#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "talosbot_custom_interfaces/action/locking_pin.hpp"
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
class LockingPinActionServer : public rclcpp::Node
{
public:
  using LockingPin = talosbot_custom_interfaces::action::LockingPin;
  using GoalHandleLockingPin = rclcpp_action::ServerGoalHandle<LockingPin>;

  TALOSBOT_ACTION_PUBLIC

  explicit LockingPinActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_locking_pin_action_server", options)
  {
    using namespace std::placeholders;

    locking_pin_command_pub_ = this->create_publisher<std_msgs::msg::Bool>("talosbot1/locking_pin_command", 10);
    local_costmap_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("local_costmap/footprint", 10);
    global_costmap_pub_ = this->create_publisher<geometry_msgs::msg::Polygon>("global_costmap/footprint", 10);


    locking_pin_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "talosbot1/locking_pin_status", 10,
        std::bind(&LockingPinActionServer::locking_pin_status_callback, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<LockingPin>(
        this, "talosbot_locking_pin", std::bind(&LockingPinActionServer::handle_goal, this, _1, _2),
        std::bind(&LockingPinActionServer::handle_cancel, this, _1),
        std::bind(&LockingPinActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<LockingPin>::SharedPtr action_server_;

  void locking_pin_status_callback(const std_msgs::msg::String& msg)
  {
    feedback_ = msg.data;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const LockingPin::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->up_position);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLockingPin> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleLockingPin> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&LockingPinActionServer::execute, this, _1), goal_handle }.detach();
  }

  void update_nav2_parameters(double xy_goal_tolerance_value, double yaw_goal_tolerance_value, double angular_accel_value){
    // Adjust the Nav2 xy_goal_tolerance parameter dynamically
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "controller_server");
    if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to parameter server!");
        return;
    }

  // Create the parameter update asynchronously
    parameters_client->set_parameters_atomically({rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", xy_goal_tolerance_value)});
    parameters_client->set_parameters_atomically({rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", yaw_goal_tolerance_value)});
    parameters_client->set_parameters_atomically({rclcpp::Parameter("FollowPath.max_angular_accel", angular_accel_value)});
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    RCLCPP_INFO(this->get_logger(), "xy_goal_tolerance set to: %f", xy_goal_tolerance_value);
    RCLCPP_INFO(this->get_logger(), "yaw_goal_tolerance set to: %f", yaw_goal_tolerance_value);
    RCLCPP_INFO(this->get_logger(), "angular_accel set to: %f", angular_accel_value);
  }

  void execute(const std::shared_ptr<GoalHandleLockingPin> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<LockingPin::Feedback>();
    auto result = std::make_shared<LockingPin::Result>();
    bool goal_achieved = false;

    //Nav2 parameters adjusment
    double xy_goal_tolerance_pinUp = 0.50;
    double yaw_goal_tolerance_pinUp = 0.25;
    double angular_accel_pinUp = 1.7;
    double xy_goal_tolerance_pinDown = 0.08;
    double yaw_goal_tolerance_pinDown = 0.03;
    double angular_accel_pinDown = 1.5;

    while (!goal_achieved)
    {
      auto command_msg = std_msgs::msg::Bool();
      if (goal->up_position)
      {
        // Move Up
        command_msg.data = true;
        if (feedback_ == "TOP")
        {
          auto robot_footprint = geometry_msgs::msg::Polygon();
          // float points[4][2] = { {0.75, 0.5}, {0.75, -0.5}, {-0.75, -0.5}, {-0.75, 0.5}}
          auto point1 = geometry_msgs::msg::Point32();
          point1.x = 0.65;
          point1.y = 0.65;
          auto point2 = geometry_msgs::msg::Point32();
          point2.x = 0.65;
          point2.y = -0.65;
          auto point3 = geometry_msgs::msg::Point32();
          point3.x = -0.65;
          point3.y = -0.65;
          auto point4 = geometry_msgs::msg::Point32();
          point4.x = -0.65;
          point4.y = 0.65;

          robot_footprint.points = {point1, point2, point3, point4};
          local_costmap_pub_->publish(robot_footprint);
          global_costmap_pub_->publish(robot_footprint);

          update_nav2_parameters(xy_goal_tolerance_pinUp, yaw_goal_tolerance_pinUp, angular_accel_pinUp);

          result->success = true;
          goal_achieved = true;
        }
      }
      else
      {
        // Move Down
        command_msg.data = false;
        if (feedback_ == "BOTTOM")
        {
          auto robot_footprint = geometry_msgs::msg::Polygon();
          // float points[4][2] = { {0.75, 0.5}, {0.75, -0.5}, {-0.75, -0.5}, {-0.75, 0.5}}
          auto point1 = geometry_msgs::msg::Point32();
          point1.x = 0.65;
          point1.y = 0.45;
          auto point2 = geometry_msgs::msg::Point32();
          point2.x = 0.65;
          point2.y = -0.45;
          auto point3 = geometry_msgs::msg::Point32();
          point3.x = -0.65;
          point3.y = -0.45;
          auto point4 = geometry_msgs::msg::Point32();
          point4.x = -0.65;
          point4.y = 0.45;

          robot_footprint.points = {point1, point2, point3, point4};
          local_costmap_pub_->publish(robot_footprint);
          global_costmap_pub_->publish(robot_footprint);

          update_nav2_parameters(xy_goal_tolerance_pinDown, yaw_goal_tolerance_pinDown, angular_accel_pinDown);

          result->success = true;
          goal_achieved = true;
        }
      }
      locking_pin_command_pub_->publish(command_msg);
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr locking_pin_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr local_costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr global_costmap_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr locking_pin_status_sub_;
  std::string feedback_ = "INVALID";
};
}  // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::LockingPinActionServer)