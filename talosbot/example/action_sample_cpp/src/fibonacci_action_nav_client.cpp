// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <sstream>

// #include "action_sample_interfaces/action/fibonacci.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

// #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_sample_cpp/visibility_control.h"

namespace action_sample_cpp
{
class FibonacciActionNavClient : public rclcpp::Node
{
public:
  using N2P = nav2_msgs::action::NavigateToPose;
  using GoalHandleN2P = rclcpp_action::ClientGoalHandle<N2P>;

  ACTION_SAMPLE_CPP_PUBLIC
  explicit FibonacciActionNavClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("fibonacci_action_nav_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<N2P>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionNavClient::send_goal, this));
  }

  ACTION_SAMPLE_CPP_PUBLIC
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = N2P::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 1.50;
    goal_msg.pose.pose.position.y = 2.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<N2P>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionNavClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionNavClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionNavClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<N2P>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  ACTION_SAMPLE_CPP_LOCAL
  void goal_response_callback(GoalHandleN2P::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  ACTION_SAMPLE_CPP_LOCAL
  void feedback_callback(
    GoalHandleN2P::SharedPtr,
    const std::shared_ptr<const N2P::Feedback> feedback)
  {
    std::stringstream ss;
    // ss << "Dist remaining: " << feedback->distance_remaining << "m; Time remaining: " << feedback->estimated_time_remaining;
    // for (auto number : feedback->distance_remaining) {
    //   ss << number << " ";
    // }
    ss << "Dist remaining: " << feedback->distance_remaining << "m";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  ACTION_SAMPLE_CPP_LOCAL
  void result_callback(const GoalHandleN2P::WrappedResult & result)
  {
    bool success = false;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        success = true;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    if(success){
      RCLCPP_INFO(this->get_logger(), "Mission Success");
    }
    rclcpp::shutdown();
  }
};  // class FibonacciActionNavClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_sample_cpp::FibonacciActionNavClient)
