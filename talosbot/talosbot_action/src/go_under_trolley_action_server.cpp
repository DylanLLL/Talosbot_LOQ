#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>

#include "talosbot_custom_interfaces/action/go_under_trolley.hpp"
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
  class GoUnderTrolleyActionServer : public rclcpp::Node
  {
  public:
    using GoUnderTrolley = talosbot_custom_interfaces::action::GoUnderTrolley;
    using GoalHandleGoUnderTrolley = rclcpp_action::ServerGoalHandle<GoUnderTrolley>;

    TALOSBOT_ACTION_PUBLIC

    explicit GoUnderTrolleyActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("talosbot_go_under_trolley_action_server", options)
    {
      using namespace std::placeholders;
      initialize_params();
      refresh_params();
      unfiltered_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/cloud_action",
          rclcpp::SensorDataQoS(), 
          std::bind(&GoUnderTrolleyActionServer::cloud_sub_callback, this, std::placeholders::_1));
      filtered_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_cloud", 10);
      target_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_cloud", 10);

      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      this->action_server_ = rclcpp_action::create_server<GoUnderTrolley>(
          this, "talosbot_go_under_trolley", std::bind(&GoUnderTrolleyActionServer::handle_goal, this, _1, _2),
          std::bind(&GoUnderTrolleyActionServer::handle_cancel, this, _1),
          std::bind(&GoUnderTrolleyActionServer::handle_accepted, this, _1));
    }

  private:
    rclcpp_action::Server<GoUnderTrolley>::SharedPtr action_server_;
    void cloud_sub_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      refresh_params();
      // if(startDetection_){
      // if (1)
      // {
        // converting ros2 pointcloud message type to pcl pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud_in);

        // creating kd tree for the pointcloud
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        cloud_in->is_dense = false;
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
        tree->setInputCloud(cloud_in);

        //RCLCPP_INFO(this->get_logger(), "Created Tree");

        // clustering points in point cloud for identifying the legs of items to pickup
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
         //RCLCPP_INFO(this->get_logger(), "Created Tree - %d %d %f", minClusterSize_, maxClusterSize_, clusterTolerance_);
        ece.setClusterTolerance(clusterTolerance_); // 20cm
        ece.setMinClusterSize(minClusterSize_);
        ece.setMaxClusterSize(maxClusterSize_);
        ece.setSearchMethod(tree);
        ece.setInputCloud(cloud_in);

        // try
        // {
        ece.extract(cluster_indices);
         //RCLCPP_INFO(this->get_logger(), "ece indices %ld", cluster_indices.size());

        // creating different point cloud instances based on clusters and adding different color
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
        for (const auto &cluster : cluster_indices)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::RGB rgb = pcl::getRandomColor();
          for (const auto &idx : cluster.indices)
          {
            (*cloud_in)[idx].r = rgb.r;
            (*cloud_in)[idx].g = rgb.g;
            (*cloud_in)[idx].b = rgb.b;
            cloud_cluster->push_back((*cloud_in)[idx]);
          }
          cloud_clusters.push_back(cloud_cluster);
        }

        std::vector<std::pair<float, float>> length_points;
        std::vector<std::pair<float, float>> breadth_points;

        for (long unsigned int i = 0; i < cloud_clusters.size(); i++)
        {
          for (long unsigned int j = i + 1; j < cloud_clusters.size(); j++)
          {
            if (cloud_clusters.at(i)->size() == 0 || cloud_clusters.at(j)->size() == 0)
            {
              break;
            }

            float y1 = std::numeric_limits<float>::max();
            float x1 = std::numeric_limits<float>::max();
            float y2 = std::numeric_limits<float>::max();
            float x2 = std::numeric_limits<float>::max();

            for (long unsigned int k = 0; k < cloud_clusters.at(i)->size(); k++)
            {
              float tempX = -cloud_clusters.at(i)->at(k).y;
              if ((tempX) < x1)
              {
                x1 = tempX;
                y1 = cloud_clusters.at(i)->at(k).x;
              }
            }

            for (long unsigned int k = 0; k < cloud_clusters.at(j)->size(); k++)
            {
              float tempX = -cloud_clusters.at(j)->at(k).y;
              if ((tempX) < x2)
              {
                x2 = tempX;
                y2 = cloud_clusters.at(j)->at(k).x;
              }
            }

            if (
                fabs(x1) < minTrolleyDistance_ &&
                fabs(x2) < minTrolleyDistance_ &&
                fabs(y1) < 1.5 &&
                fabs(y2) < 1.5)
            {
              float distance = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
              distance = fabs(distance);

              if (distance < maxTrolleyGap_ && distance > minTrolleyGap_)
              {
                 //RCLCPP_INFO(this->get_logger(), "distance %f", distance);
                std::pair<float, float> first_point_pair;
                first_point_pair.first = x1;
                first_point_pair.second = y1;

                std::pair<float, float> second_point_pair;
                second_point_pair.first = x2;
                second_point_pair.second = y2;

                length_points.push_back(first_point_pair);
                length_points.push_back(second_point_pair);
              }
            }
          }
        }

         //RCLCPP_INFO(this->get_logger(), "length %ld", length_points.size());
         //RCLCPP_INFO(this->get_logger(), "breadth %ld", breadth_points.size());
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        pcl::PointXYZRGB pt;

        if (length_points.size() == 2)
        {
          float x1 = ((length_points.at(0).first + length_points.at(1).first) / 2);
          float y1 = ((length_points.at(0).second + length_points.at(1).second) / 2);

          if (fabs(x1) < minTrolleyDistance_)
          {
            pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)255, (uint8_t)0);
            pt.x = x1;
            pt.y = y1;
            cloud_.points.push_back(pt);
            pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)0, (uint8_t)255);
            pt.x = length_points.at(0).first;
            pt.y = length_points.at(0).second;
            cloud_.points.push_back(pt);
            pt.x = length_points.at(1).first;
            pt.y = length_points.at(1).second;
            cloud_.points.push_back(pt);
            // RCLCPP_INFO(this->get_logger(), "cent_x %f", x1);
            // RCLCPP_INFO(this->get_logger(), "cent_y %f", y1);,
            target_x_ = x1;
            // target_y_ = y1 - 0.05;
            target_y_ = y1;

            if (length_points.at(0).second > length_points.at(1).second)
            {
              target_y_diff_ = length_points.at(0).first - length_points.at(1).first;
            }
            else
            {
              target_y_diff_ = length_points.at(1).first - length_points.at(0).first;
            }

            float dx = target_y_diff_ * prePositionDistance_ / trolleyGap_;
            float dy = (std::sqrt(prePositionDistance_ * prePositionDistance_ - dx * dx));

            pre_target_x_ = x1 - dy;
            pre_target_y_ = y1 + dx;

            float dx_final = target_y_diff_ * postMovementOffset_ / trolleyGap_;
            float dy_final = (std::sqrt(postMovementOffset_ * postMovementOffset_ - dx_final * dx_final));

            final_target_x_ = x1 + dy_final;
            final_target_y_ = y1 - dx_final;

            // RCLCPP_INFO(this->get_logger(), "pre_cent_x %f", pre_target_x_);
            // RCLCPP_INFO(this->get_logger(), "pre_cent_y %f", pre_target_y_);
            pt = pcl::PointXYZRGB((uint8_t)255, (uint8_t)0, (uint8_t)255);
            pt.x = pre_target_x_;
            pt.y = pre_target_y_;
            cloud_.points.push_back(pt);
            pt = pcl::PointXYZRGB((uint8_t)255, (uint8_t)255, (uint8_t)0);
            pt.x = final_target_x_;
            pt.y = final_target_y_;
            cloud_.points.push_back(pt);
            pt = pcl::PointXYZRGB((uint8_t)0, (uint8_t)255, (uint8_t)255);
            pt.x = 0;
            pt.y = 0;
            cloud_.points.push_back(pt);
          }
        }
        else
        {
          target_x_ = NULL;
          target_y_ = NULL;
          pre_target_x_ = NULL;
          pre_target_y_ = NULL;
          final_target_x_ = NULL;
          final_target_y_ = NULL;
        }

        auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msg_);
        pc2_msg_->header.frame_id = "base_footprint";
        pc2_msg_->header.stamp = now();
        pc2_msg_->is_dense = false;
        target_cloud_pub_->publish(*pc2_msg_);
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_out(new sensor_msgs::msg::PointCloud2());
        pcl::toROSMsg(*cloud_in, *cloud_out);
        cloud_out->header.frame_id = msg->header.frame_id;
        cloud_out->header.stamp = msg->header.stamp;
        filtered_cloud_->publish(*cloud_out);
      // }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const GoUnderTrolley::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->go_under_trolley);
      startDetection_ = true;
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoUnderTrolley> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoUnderTrolley> goal_handle)
    {
      using namespace std::placeholders;
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&GoUnderTrolleyActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoUnderTrolley> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(50);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<GoUnderTrolley::Feedback>();
      auto result = std::make_shared<GoUnderTrolley::Result>();
      bool goal_achieved = false;
      int failed_count = 0;
      pre_position_achieved_ = false;
      heading_achieved_ = false;
      float actual_speed_ = 0;

      int state = 0;
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
              state = 10;
            }
            else
            {
              if (fabs(pre_target_y_) >= coarsePositioningTolerance_)
              {
                vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
                actual_speed_ = filter_linear_velocity(0, actual_speed_);
                vel_msg_.linear.x = actual_speed_;
              }
              else
              {
                if (pre_target_x_ > 0)
                {
                  actual_speed_ = filter_linear_velocity(movementLinearVel_, actual_speed_);
                  vel_msg_.linear.x = actual_speed_;
                }
                else
                {
                  // vel_msg_.linear.x = -movementLinearVel_;
                }
                vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
              }
              if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
                vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
              cmd_pub_->publish(vel_msg_);
            }
            break;

          // Align Heading
          case 10:
            if (fabs(target_y_diff_) >= finePositioningTolerance_)
            {
              actual_speed_ = filter_linear_velocity(0, actual_speed_);
              vel_msg_.linear.x = actual_speed_;
              vel_msg_.angular.z = -target_y_diff_ * angularVelGain_;
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
            if ((final_target_x_ <= finePositioningTolerance_) && (fabs(final_target_y_) < finePositioningTolerance_))
            {
              cmd_pub_->publish(vel_msg_);
              goal_reached_count_++;
              if (goal_reached_count_ >= 40)
              {
                result->success = true;
                goal_achieved = true;
              }
            }
            else
            {
              if (fabs(final_target_y_) >= finePositioningTolerance_)
              {
                vel_msg_.angular.z = final_target_y_ * angularVelGain_;
                actual_speed_ = filter_linear_velocity(0, actual_speed_);
                vel_msg_.linear.x = actual_speed_;
              }
              else
              {
                if (final_target_x_ > 0)
                {
                  actual_speed_ = filter_linear_velocity(movementLinearVel_, actual_speed_);
                  vel_msg_.linear.x = actual_speed_;
                }
                else
                {
                  // vel_msg_.linear.x = -movementLinearVel_;
                }
                vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
              }
              if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
                vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
              cmd_pub_->publish(vel_msg_);
            }
            break;
          default:
            break;
          }

          // if (pre_position_achieved_)
          // {
          //   if (heading_achieved_)
          //   {
          //     if (target_x_ <= -postMovementOffset_)
          //     {
          //       cmd_pub_->publish(vel_msg_);
          //       goal_reached_count_++;
          //       if (goal_reached_count_ >= 40)
          //       {
          //         result->success = true;
          //         goal_achieved = true;
          //       }
          //     }
          //     else
          //     {
          //       actual_speed_ = filter_linear_velocity(movementLinearVel_, actual_speed_);
          //       vel_msg_.linear.x = actual_speed_;
          //       if (fabs(target_y_) >= finePositioningTolerance_)
          //       {
          //         vel_msg_.angular.z = target_y_ * angularVelGain_;
          //       }
          //       else
          //       {
          //         vel_msg_.angular.z = -target_y_diff_ * angularVelGain_;
          //       }
          //       if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
          //         vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
          //       cmd_pub_->publish(vel_msg_);
          //     }
          //   }
          //   else
          //   {
          //     if (fabs(target_y_diff_) >= finePositioningTolerance_)
          //     {
          //       actual_speed_ = filter_linear_velocity(0, actual_speed_);
          //       vel_msg_.linear.x = actual_speed_;
          //       vel_msg_.angular.z = -target_y_diff_ * angularVelGain_;
          //       if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
          //         vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
          //       cmd_pub_->publish(vel_msg_);
          //     }
          //     else
          //     {
          //       cmd_pub_->publish(vel_msg_);
          //       heading_achieved_ = true;
          //     }
          //   }
          // }
          // else
          // {
          //   if ((pre_target_x_ <= coarsePositioningTolerance_) && (fabs(pre_target_y_) < coarsePositioningTolerance_))
          //   {
          //     cmd_pub_->publish(vel_msg_);
          //     pre_position_achieved_ = true;
          //   }
          //   else
          //   {
          //     if (fabs(pre_target_y_) >= coarsePositioningTolerance_)
          //     {,
          //       vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
          //       actual_speed_ = filter_linear_velocity(0, actual_speed_);
          //       vel_msg_.linear.x = actual_speed_;
          //     }
          //     else
          //     {
          //       if (pre_target_x_ > 0)
          //       {
          //         actual_speed_ = filter_linear_velocity(movementLinearVel_, actual_speed_);
          //         vel_msg_.linear.x = actual_speed_;,
          //       }
          //       else
          //       {
          //         // vel_msg_.linear.x = -movementLinearVel_;
          //       }
          //       vel_msg_.angular.z = pre_target_y_ * angularVelGain_;
          //     }
          //     if (fabs(vel_msg_.angular.z) >= maxAngularVel_)
          //       vel_msg_.angular.z = maxAngularVel_ * (fabs(vel_msg_.angular.z) / vel_msg_.angular.z);
          //     cmd_pub_->publish(vel_msg_);
          //   }
          // }
        }
        else
        {
          failed_count++;
        }

        startDetection_ = false;

        if (failed_count >= (30000 / 50))
        {
          feedback->status = "NO Trolley Found";
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
      // Euclidean Cluster Extraction Related
      this->declare_parameter("minClusterSize", 5); //5
      this->declare_parameter("maxClusterSize", 150); //150
      this->declare_parameter("clusterTolerance", 0.50); //0.80

      // Trolley Related
      this->declare_parameter("minTrolleyDistance", 2.50); //2.50
      this->declare_parameter("minTrolleyGap", 0.70); //0.85
      this->declare_parameter("maxTrolleyGap", 1.20); //1.2
      this->declare_parameter("trolleyGap", 1.10); //1.10

      // Movement Related
      this->declare_parameter("prePositionDistance", 1.0); //1.0
      this->declare_parameter("postMovementOffset", 0.35); //0.35
      this->declare_parameter("movementLinearVel", 0.12); //0.12
      this->declare_parameter("maxAngularVel", 0.15); //0.15
      this->declare_parameter("angularVelGain", 2.5); //2.5
      this->declare_parameter("coarsePositioningTolerance", 0.03); //0.03
      this->declare_parameter("finePositioningTolerance", 0.03); //0.03
    }

    void refresh_params()
    {
      // Euclidean Cluster Extraction Related
      this->get_parameter_or<int>("minClusterSize", minClusterSize_, 5); //5
      this->get_parameter_or<int>("maxClusterSize", maxClusterSize_, 150); //150
      this->get_parameter_or<float>("clusterTolerance", clusterTolerance_, 0.50); //0.80

      // Trolley Related
      this->get_parameter_or<float>("minTrolleyDistance", minTrolleyDistance_, 2.50); //2.50
      this->get_parameter_or<float>("minTrolleyGap", minTrolleyGap_, 0.70); //0.85
      this->get_parameter_or<float>("maxTrolleyGap", maxTrolleyGap_, 1.20); //1.20
      this->get_parameter_or<float>("trolleyGap", trolleyGap_, 1.10); //1.10

      // Movement Related
      this->get_parameter_or<float>("prePositionDistance", prePositionDistance_, 1.0); //1.0
      this->get_parameter_or<float>("postMovementOffset", postMovementOffset_, 0.35); //0.35
      this->get_parameter_or<float>("movementLinearVel", movementLinearVel_, 0.12); //0.12
      this->get_parameter_or<float>("maxAngularVel", maxAngularVel_, 0.15); //0.15
      this->get_parameter_or<float>("angularVelGain", angularVelGain_, 2.5); //2.5
      this->get_parameter_or<float>("coarsePositioningTolerance", coarsePositioningTolerance_, 0.03); //0.10
      this->get_parameter_or<float>("finePositioningTolerance", finePositioningTolerance_, 0.03); //0.03
    }

    // Params Var
    int minClusterSize_;
    int maxClusterSize_;
    float clusterTolerance_;

    float minTrolleyDistance_;
    float minTrolleyGap_;
    float maxTrolleyGap_;
    float trolleyGap_;

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

    bool startDetection_ = true;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_cloud_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::string feedback_ = "INVALID";
  };
} // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::GoUnderTrolleyActionServer)