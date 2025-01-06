#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>

#include "talosbot_custom_interfaces/action/go_middle_front_door.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/colors.h>

#include "talosbot_action/visibility_control.h"

namespace talosbot_action
{
  class GoMiddleFrontDoorActionServer : public rclcpp::Node
  {
  public:
    using GoMiddleFrontDoor = talosbot_custom_interfaces::action::GoMiddleFrontDoor;
    using GoalHandleGoMiddleFrontDoor = rclcpp_action::ServerGoalHandle<GoMiddleFrontDoor>;

    TALOSBOT_ACTION_PUBLIC

    explicit GoMiddleFrontDoorActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("talosbot_go_middle_front_door_action_server", options)
    {
      using namespace std::placeholders;
      initialize_params();
      refresh_params();
      laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", 10,
          std::bind(&GoMiddleFrontDoorActionServer::laser_sub_callback, this, std::placeholders::_1));
      filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/door_filtered_cloud", 10);
      target_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/door_target_cloud", 10);
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      this->action_server_ = rclcpp_action::create_server<GoMiddleFrontDoor>(
          this, "talosbot_go_middle_front_door", std::bind(&GoMiddleFrontDoorActionServer::handle_goal, this, _1, _2),
          std::bind(&GoMiddleFrontDoorActionServer::handle_cancel, this, _1),
          std::bind(&GoMiddleFrontDoorActionServer::handle_accepted, this, _1));
    }

  private:
    rclcpp_action::Server<GoMiddleFrontDoor>::SharedPtr action_server_;
    void laser_sub_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      refresh_params();
      //RCLCPP_INFO(this->get_logger(), "Starting detection")
      pcl::PointCloud<pcl::PointXYZRGB> cloud_;
      pcl::PointCloud<pcl::PointXYZRGB> target_cloud_;
      
      
      unsigned long laser_scan_size_ = msg->ranges.size();
      double min_angle_ = msg->angle_min;
      double angle_increment_ = msg->angle_increment;
      // double temp_arr[laser_scan_size_][2];

      std::vector<std::array<double, 2>> valid_scan;
      pcl::PointXYZRGB pt;
      pt = pcl::PointXYZRGB((uint8_t) 0, (uint8_t) 255, (uint8_t) 0);
      for (unsigned long i = 0; i < laser_scan_size_; i++)
      {
        double temp_range = msg->ranges[i];
        double temp_angle = min_angle_ + i * angle_increment_;

        if((temp_range >= min_laser_range) && (temp_range <= max_laser_range)){
          double temp_x = -temp_range * std::sin(temp_angle);
          double temp_y = temp_range * std::cos(temp_angle);

          valid_scan.push_back({temp_x, temp_y});

          // pt.x = temp_x;
          // pt.y = temp_y;

          // cloud_.points.push_back(pt);
        }
      }

      //THIS SUPPOSED TO BE ON
      //RCLCPP_INFO(this->get_logger(), "Receive Scan, %f %f", min_laser_range, max_laser_range);
      //RCLCPP_INFO(this->get_logger(), "Valid Scan: %ld", valid_scan.size());
      
      std::vector<std::array<double, 4>> valid_points;

      unsigned long skip_beam_ = 4;
      
      double gap_width = 1.0;

      for (unsigned long i = 0; i < valid_scan.size(); i++)
      {
        for (unsigned long j = i + 1;  j< valid_scan.size(); j++)
        {
          double x1 = valid_scan[i][0];
          double y1 = valid_scan[i][1];
          double x2 = valid_scan[j][0];
          double y2 = valid_scan[j][1];

          double x_mid = (x1 + x2) / 2;
          double y_mid = (y1 + y2) / 2;

          // Check Center
          if(std::abs(y_mid) < 1.0){
            // Check Gap
            bool has_gap = true;
            for (unsigned long k = i + 1; k < j-1; k++)
            {
              double temp_x = valid_scan[k][0];
              double temp_y = valid_scan[k][1];
              if(std::hypot(temp_x - x_mid, temp_y - y_mid) < 0.8 * doorGap){
                has_gap = false;
                break;
              }
            }

            if(has_gap){
              double dist = std::hypot(x2 - x1, y2 - y1);
              if (std::abs(dist - doorGap) < 0.2)
              {
                pcl::RGB rgb = pcl::getRandomColor();
                pt = pcl::PointXYZRGB((uint8_t) rgb.r, (uint8_t) rgb.g, (uint8_t) rgb.b);
                pt.x = x1;
                pt.y = y1;
                cloud_.points.push_back(pt);
                pt.x = x2;
                pt.y = y2;
                cloud_.points.push_back(pt);
                valid_points.push_back({x1, y1, x2, y2});
              }
            }
          }
        }
      }

      //THIS SUPPOSED TO BE ON
      //RCLCPP_INFO(this->get_logger(), "Valid Points: %ld", valid_points.size());
      if(valid_points.size() > 0){
        double chosen_x_1, chosen_y_1, chosen_x_2, chosen_y_2;
        double min_y = std::numeric_limits<double>::max();
        for (int i = 0; i < valid_points.size(); i++)
        {
          double x1 = valid_points[i][0];
          double y1 = valid_points[i][1];
          double x2 = valid_points[i][2];
          double y2 = valid_points[i][3];
          
          double alignmentWeight = fabs(x1-x2);
          double gapWeight = fabs(std::hypot(x2-x1,y2-y1) - doorGap);
          double currentWeight = door_alignment_weight * alignmentWeight + door_gap_weight * gapWeight;
          if(currentWeight < min_y){
            min_y = currentWeight;
            chosen_x_1 = x1;
            chosen_y_1 = y1;
            chosen_x_2 = x2;
            chosen_y_2 = y2;  
          }
          // if(((x1+x2)/2 < min_y) && (fabs(x1-x2) < 0.4)){
          //   min_y = (x1 + x2) / 2;
          //   chosen_x_1 = x1;
          //   chosen_y_1 = y1;
          //   chosen_x_2 = x2;
          //   chosen_y_2 = y2;
          // }
        }
        target_y_ = ((chosen_y_1 + chosen_y_2) /2) - 0.02;
        target_x_ = (chosen_x_1 + chosen_x_2) /2;
        if(prevTargetX != NULL && prevTargetY != NULL){
          target_y_ = target_filter_ratio * prevTargetY + (1-target_filter_ratio) * target_y_;
          target_x_ = target_filter_ratio * prevTargetX + (1-target_filter_ratio) * target_x_;
          prevTargetX = target_x_;
          prevTargetY = target_y_;
        }

        if(chosen_y_1 < chosen_y_2){
          target_y_diff_ = chosen_x_1 - chosen_x_2;
        }else{
          target_y_diff_ = chosen_x_2 - chosen_x_1;
        }

        float dx = target_y_diff_ * prePositionDistance_ / doorGap;
        float dy = (std::sqrt(prePositionDistance_ * prePositionDistance_ - dx * dx));

        pre_target_x_ = target_x_ - dy;
        pre_target_y_ = target_y_ - dx;

        float dx_final = target_y_diff_ * postMovementOffset_ / doorGap;
        float dy_final = (std::sqrt(postMovementOffset_ * postMovementOffset_ - dx_final * dx_final));

        final_target_x_ = target_x_ - dy_final;
        final_target_y_ = target_y_ - dx_final;

        pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)0, (uint8_t)255);
        pt.x = chosen_x_1;
        pt.y = chosen_y_1;
        target_cloud_.points.push_back(pt);
        pt.x = chosen_x_2;
        pt.y = chosen_y_2;
        target_cloud_.points.push_back(pt);
        pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)0, (uint8_t)255);
        pt.x = target_x_;
        pt.y = target_y_;
        target_cloud_.points.push_back(pt);
        pt = pcl::PointXYZRGB((uint8_t)255, (uint8_t)0, (uint8_t)255);
        pt.x = pre_target_x_;
        pt.y = pre_target_y_;
        target_cloud_.points.push_back(pt);
        pt = pcl::PointXYZRGB((uint8_t)255, (uint8_t)255, (uint8_t)0);
        pt.x = final_target_x_;
        pt.y = final_target_y_;
        target_cloud_.points.push_back(pt);
        pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)255, (uint8_t)255);
        pt.x = 0;
        pt.y = 0;
        target_cloud_.points.push_back(pt);
      }

      auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(cloud_, *pc2_msg_);
      pc2_msg_->header.frame_id = "base_footprint";
      pc2_msg_->header.stamp = now();
      pc2_msg_->is_dense = false;
      filtered_cloud_->publish(*pc2_msg_);

      pcl::toROSMsg(target_cloud_, *pc2_msg_);
      pc2_msg_->header.frame_id = "base_footprint";
      pc2_msg_->header.stamp = now();
      pc2_msg_->is_dense = false;
      target_cloud_pub_->publish(*pc2_msg_);
   }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const GoMiddleFrontDoor::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->go_middle_front_door);
      startDetection_ = true;
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoMiddleFrontDoor> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoMiddleFrontDoor> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&GoMiddleFrontDoorActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoMiddleFrontDoor> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(50);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<GoMiddleFrontDoor::Feedback>();
      auto result = std::make_shared<GoMiddleFrontDoor::Result>();
      bool goal_achieved = false;
      int failed_count = 0;
      pre_position_achieved_ = false;
      heading_achieved_ = false;
      float actual_speed_ = 0;
      goal_reached_count_=0;
      int state = 0;
      prevTargetX = target_x_;
      prevTargetY = target_y_;

      while (!goal_achieved)
      {
        if ((target_x_ != NULL) && (target_y_ != NULL) && (pre_target_x_ != NULL) && (pre_target_y_ != NULL) && (final_target_x_ != NULL) && (final_target_y_ != NULL))
        {
          failed_count = 0;
          geometry_msgs::msg::Twist vel_msg_;
          vel_msg_.linear.x = 0.00;
          vel_msg_.linear.y = 0.00;
          vel_msg_.angular.z = 0.00;

          switch (state)
          {
          // Go to Pre Position
          case 0:
            if ((pre_target_x_ <= coarsePositioningTolerance_) && (fabs(pre_target_y_) < coarsePositioningTolerance_))
            {
              cmd_pub_->publish(vel_msg_);
              pre_position_achieved_ = true;
              state = 20;
            }
            else
            {
              double angular_vel = 0.2 * std::atan2(pre_target_y_, pre_target_x_);
              double linear_vel = 0.20 * (std::sqrt(pre_target_x_ * pre_target_x_) + 0.05);
              vel_msg_.angular.z = angular_vel;
              vel_msg_.linear.x = linear_vel;
              // if (fabs(pre_target_y_) >= coarsePositioningTolerance_)
              // {
              //   vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
              //   actual_speed_ = filter_linear_velocity(0, actual_speed_);
              //   vel_msg_.linear.x = actual_speed_;
              // }
              // else
              // {
              //   if (pre_target_x_ > 0)
              //   {
              //     actual_speed_ = filter_linear_velocity(movementLinearVel_, actual_speed_);
              //     vel_msg_.linear.x = actual_speed_;
              //   }
              //   else
              //   {
              //     // vel_msg_.linear.x = -movementLinearVel_;
              //   }
              //   vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
              // }
              // if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
              //   vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
              cmd_pub_->publish(vel_msg_);
            }
            break;

          // Align Heading
          case 10:
            if (fabs(target_y_diff_) >= coarsePositioningTolerance_)
            {
              actual_speed_ = filter_linear_velocity(0, actual_speed_);
              vel_msg_.linear.x = actual_speed_;
              vel_msg_.angular.z = target_y_diff_ * angularVelGain_;
              if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
                vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
              cmd_pub_->publish(vel_msg_);
            }
            else
            {
              cmd_pub_->publish(vel_msg_);
              heading_achieved_ = true;
              state = 20;
            }
            break;

          // Go to Final Pos
          case 20:
            // if ((final_target_x_ <= finePositioningTolerance_) && (fabs(final_target_y_) < finePositioningTolerance_))
            if ((final_target_x_ <= finePositioningTolerance_))
            {
              cmd_pub_->publish(vel_msg_);
              goal_reached_count_++;
              if (goal_reached_count_ >= 40)
              {
                state = 30;
                goal_reached_count_ = 0;
              }
            }
            else
            {
              goal_reached_count_ = 0;
              double angular_vel = 0.2 * std::atan2(final_target_y_, final_target_x_);
              double linear_vel = 0.15 * (std::sqrt(final_target_x_ * final_target_x_) + 0.05);
              vel_msg_.angular.z = angular_vel;
              vel_msg_.linear.x = linear_vel;
              // if (fabs(final_target_y_) >= finePositioningTolerance_)
              // {
              //   vel_msg_.angular.z = final_target_y_ * angularVelGain_;
              //   actual_speed_ = filter_linear_velocity(0, actual_speed_);
              //   vel_msg_.linear.x = actual_speed_;
              // }
              // else
              // {
              //   if (final_target_x_ > 0)
              //   {
              //     if(final_target_x_ <= 0.15){
              //       actual_speed_ = filter_linear_velocity(movementLinearVel_ * 1/2, actual_speed_);
              //     }else{
              //       actual_speed_ = filter_linear_velocity(movementLinearVel_, actual_speed_);
              //     }
              //     vel_msg_.linear.x = actual_speed_;
              //   }
              //   else
              //   {
              //     // vel_msg_.linear.x = -movementLinearVel_;
              //   }
              //   vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
              // }
              // if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
              //   vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
              cmd_pub_->publish(vel_msg_);
            }
            break;
          // Adjust final Heading
          case 30:
            if (fabs(target_y_diff_) >= finePositioningTolerance_ / 3)
            {
              goal_reached_count_ = 0;
              actual_speed_ = filter_linear_velocity(0, actual_speed_);
              vel_msg_.linear.x = actual_speed_;
              vel_msg_.angular.z = target_y_diff_ * angularVelGain_;
              if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
                vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
              cmd_pub_->publish(vel_msg_);
            }
            else
            {
              cmd_pub_->publish(vel_msg_);
              goal_reached_count_++;
              if (goal_reached_count_ >= 40)
              {
                prevTargetX = NULL;
                prevTargetY = NULL;
                result->success = true;
                goal_achieved = true;
              }
            }
            break;

          default:
            break;
          }
        }
        else
        {
          failed_count++;
        }

        startDetection_ = false;

        if (failed_count >= (30000 / 50))
        {
          feedback->status = "NO Door Found";
          // goal_handle->publish_feedback(feedback);
          result->success = false;
          goal_achieved = true;
        }

        if (goal_handle->is_canceling())
        {

          geometry_msgs::msg::Twist vel_msg_;
          vel_msg_.linear.x = 0.00;
          vel_msg_.linear.y = 0.00;
          vel_msg_.angular.z = 0.00;
          cmd_pub_->publish(vel_msg_);
          result->success = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }

        // feedback->status = "Ongoing";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
      }

      if (rclcpp::ok())
      {
        if (result->success)
        {
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal achieved");
        }
        else
        {
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Mission Aborted");
        }
      }
    }
    float filter_linear_velocity(float target, float actual)
    {
      if (target != actual)
      {
        if (fabs(actual - target) < 0.005)
        {
          return target;
        }
        if (target < actual)
        {
          return actual - 0.002;
        }
        if (target > actual)
        {
          return actual + 0.002;
        }
      }
    }
    void initialize_params()
    {
      // Laser Scan Related
      this->declare_parameter("min_laser_range", 0.5);
      this->declare_parameter("max_laser_range", 10.0);

      // Door Related
      this->declare_parameter("doorGap", 0.94);

      // Filter Related
      this->declare_parameter("target_filter_ratio", 0.6);

      // Movement Related
      this->declare_parameter("prePositionDistance", 1.7);
      this->declare_parameter("postMovementOffset", 1.1);
      this->declare_parameter("movementLinearVel", 0.15);
      this->declare_parameter("maxAngularVel", 0.25);
      this->declare_parameter("angularVelGain", 2.5);
      this->declare_parameter("coarsePositioningTolerance", 0.10);
      this->declare_parameter("finePositioningTolerance", 0.03);
    }

    void refresh_params()
    { 
      // Laser Scan Related
      this->get_parameter_or<double>("min_laser_range", min_laser_range, 0.5);
      this->get_parameter_or<double>("max_laser_range", max_laser_range, 10.0);
      
      // Door Related
      this->get_parameter_or<double>("doorGap", doorGap, 0.94);
      
      // Filter Related
      this->get_parameter_or<double>("target_filter_ratio",target_filter_ratio, 0.6);
      
      // Movement Related
      this->get_parameter_or<float>("prePositionDistance", prePositionDistance_, 1.7);
      this->get_parameter_or<float>("postMovementOffset", postMovementOffset_, 1.1);
      this->get_parameter_or<float>("movementLinearVel", movementLinearVel_, 0.15);
      this->get_parameter_or<float>("maxAngularVel", maxAngularVel_, 0.25);
      this->get_parameter_or<float>("angularVelGain", angularVelGain_, 2.5);
      this->get_parameter_or<float>("coarsePositioningTolerance", coarsePositioningTolerance_, 0.10);
      this->get_parameter_or<float>("finePositioningTolerance", finePositioningTolerance_, 0.03);
    }

    // Params Var
    double min_laser_range, max_laser_range;
    double doorGap;
    double prevTargetX, prevTargetY = NULL;

    double target_filter_ratio; 
    double door_alignment_weight, door_gap_weight = 1.0;
    float prePositionDistance_;
    float postMovementOffset_;
    float movementLinearVel_;
    float maxAngularVel_;
    float angularVelGain_;
    float coarsePositioningTolerance_;
    float finePositioningTolerance_;

    bool pre_position_achieved_ = false;
    bool heading_achieved_ = false;
    float pre_target_x_ = NULL;
    float pre_target_y_ = NULL;
    float final_target_x_ = NULL;
    float final_target_y_ = NULL;

    int timeout_count_ = 0;
    float target_x_ = NULL;
    float target_y_ = NULL;
    float target_y_diff_ = NULL;
    int goal_reached_count_ = 0;

    bool startDetection_ = false;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::string feedback_ = "INVALID";
  };
} // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::GoMiddleFrontDoorActionServer)