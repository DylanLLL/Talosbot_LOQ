//normal cpp library
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <math.h>
#include <iostream>

//library for ros2 and ros2 action
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

//library to access topic messages
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

//library to talosbot custom interfaces
#include "talosbot_custom_interfaces/action/template.hpp"

//library to make the action be used as "ros2 action send_goal ..."
#include "rclcpp_components/register_node_macro.hpp"

namespace talosbot_action
{
    class TemplateActionServer : public rclcpp::Node
    {
        public:
        using Template = talosbot_custom_interfaces::action::Template;
        using GoalHandleTemplate = rclcpp_action::ServerGoalHandle<Template>;

        explicit TemplateActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("talosbot_template_action_server", options)
        {
            action_server_ = rclcpp_action::create_server<Template>(
                this,
                "talosbot_template",
                std::bind(&TemplateActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&TemplateActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&TemplateActionServer::handle_accepted, this, std::placeholders::_1));
        }

        private:
        rclcpp_action::Server<Template>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &, std::shared_ptr<const Template::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Target number is: %d", goal->num);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTemplate>)
        {
            RCLCPP_INFO(this->get_logger(), "Action is cancelled.");
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTemplate> goal_handle)
        {
            std::thread{std::bind(&TemplateActionServer::execute, this, goal_handle)}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleTemplate> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Action started.");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Template::Feedback>();
            auto result = std::make_shared<Template::Result>();
            bool goal_achieved = false;

            while(!goal_achieved)
            {
                for(int i = 0; i <= goal->num; i++)
                {
                    RCLCPP_INFO(this->get_logger(), "number %d", i);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                if (goal_handle->is_canceling())
                {
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
            }

            if(rclcpp::ok)
            {
            // Indicate success
            auto result = std::make_shared<Template::Result>();
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Action succeeded.");
            }

        }
    };

    int mainTemplate(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<TemplateActionServer>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::TemplateActionServer)
