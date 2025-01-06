#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <math.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "talosbot_custom_interfaces/action/relocalize.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace talosbot_action
{
class RelocalizeActionServer : public rclcpp::Node
{
public:
    using Relocalize = talosbot_custom_interfaces::action::Relocalize;
    using GoalHandleRelocalize = rclcpp_action::ServerGoalHandle<Relocalize>;

    explicit RelocalizeActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_relocalize_action_server", options)
    {
        // Publisher for /initialpose
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Subscriber for /odom
        amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&RelocalizeActionServer::amcl_callback, this, std::placeholders::_1));

        // Action server for re-localization
        action_server_ = rclcpp_action::create_server<Relocalize>(
            this,
            "talosbot_relocalize",
            std::bind(&RelocalizeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RelocalizeActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&RelocalizeActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
    rclcpp_action::Server<Relocalize>::SharedPtr action_server_;
    std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> current_amcl_;

    void  amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_amcl_ = msg;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const Relocalize::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Re-localization is set to %d", goal->relocalize);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRelocalize>)
    {
        RCLCPP_INFO(this->get_logger(), "Re-localization goal canceled.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleRelocalize> goal_handle)
    {
        std::thread{std::bind(&RelocalizeActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleRelocalize> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Re-localization started.");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Relocalize::Feedback>();
        auto result = std::make_shared<Relocalize::Result>();
        bool goal_achieved = false;

        // Wait for /odom data to be available
        while (!current_amcl_)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for /odom data...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        while(!goal_achieved){
            if (goal->relocalize){
                // Publish the pose to /initialpose
                geometry_msgs::msg::PoseWithCovarianceStamped relocalize_pose;
                relocalize_pose.header.stamp = this->get_clock()->now();
                relocalize_pose.header.frame_id = "map";
                relocalize_pose.pose.pose = current_amcl_->pose.pose;

                // Set covariance
                for (size_t i = 0; i < 36; ++i)
                {
                    relocalize_pose.pose.covariance[i] = 0.0;
                }
                relocalize_pose.pose.covariance[0] = 0.5;
                relocalize_pose.pose.covariance[7] = 0.5;
                relocalize_pose.pose.covariance[35] = 0.1;

                RCLCPP_INFO(this->get_logger(), "Publishing pose to /initialpose...");
                
                for (int i = 0; i < 5; ++i)
                {
                    pose_publisher_->publish(relocalize_pose);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                result->success = true;
                goal_achieved = true;
            }   
            
            if (goal_handle->is_canceling()){
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
        }

        if(rclcpp::ok){
            // Indicate success
            auto result = std::make_shared<Relocalize::Result>();
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Re-localization succeeded.");
        }
    }
};

int mainRelocalize(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizeActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::RelocalizeActionServer)