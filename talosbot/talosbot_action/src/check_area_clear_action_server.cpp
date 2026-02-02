#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <chrono>
#include <cmath>
#include <vector>
#include <limits>

#include "talosbot_custom_interfaces/action/check_area_clear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "talosbot_action/visibility_control.h"

namespace talosbot_action
{
class CheckAreaClearActionServer : public rclcpp::Node
{
public:
  using CheckAreaClear = talosbot_custom_interfaces::action::CheckAreaClear;
  using GoalHandleCheckAreaClear = rclcpp_action::ServerGoalHandle<CheckAreaClear>;

  TALOSBOT_ACTION_PUBLIC

  explicit CheckAreaClearActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("talosbot_check_area_clear_action_server", options)
  {
    using namespace std::placeholders;
    initialize_params();
    refresh_params();

    // Get the point cloud topic from parameter (default: /cloud_in for simulation compatibility)
    std::string cloud_topic;
    this->get_parameter_or<std::string>("cloudTopic", cloud_topic, "/cloud_in");
    RCLCPP_INFO(this->get_logger(), "Subscribing to point cloud topic: %s", cloud_topic.c_str());

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&CheckAreaClearActionServer::cloud_sub_callback, this, std::placeholders::_1));

    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/check_area_filtered_cloud", 10);

    this->action_server_ = rclcpp_action::create_server<CheckAreaClear>(
        this, "talosbot_check_area_clear", std::bind(&CheckAreaClearActionServer::handle_goal, this, _1, _2),
        std::bind(&CheckAreaClearActionServer::handle_cancel, this, _1),
        std::bind(&CheckAreaClearActionServer::handle_accepted, this, _1));

    // RCLCPP_INFO(this->get_logger(), "CheckAreaClear Action Server initialized");
  }

private:
  rclcpp_action::Server<CheckAreaClear>::SharedPtr action_server_;

  void cloud_sub_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!detection_active_)
    {
      return;
    }

    refresh_params();

    // Convert ROS2 PointCloud2 to PCL PointCloud (use PointXYZ instead of PointXYZRGB for compatibility)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_in);

    // Remove NaN points
    cloud_in->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);

    // Debug: Log point cloud info on first sample
    if (debug_log_count_ < 3)
    {
      RCLCPP_INFO(this->get_logger(), "Point cloud received: %zu points, frame: %s",
                  cloud_in->points.size(), msg->header.frame_id.c_str());

      // Find min/max X and Y values to understand the point cloud bounds
      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();

      // Count points in different regions for debugging
      int points_front_close = 0;   // X: 0 to 1m, any Y
      int points_front_mid = 0;     // X: 1 to 3m, any Y
      int points_front_far = 0;     // X: 3 to 10m, any Y
      int points_in_y_range = 0;    // Any X, Y: -0.6 to 0.6
      int points_x_positive = 0;    // X > 0 (in front)

      for (const auto& point : cloud_in->points)
      {
        if (std::isfinite(point.x) && std::isfinite(point.y))
        {
          min_x = std::min(min_x, point.x);
          max_x = std::max(max_x, point.x);
          min_y = std::min(min_y, point.y);
          max_y = std::max(max_y, point.y);

          // Count by region
          if (point.x > 0)
          {
            points_x_positive++;
            if (point.x <= 1.0)
              points_front_close++;
            else if (point.x <= 3.0)
              points_front_mid++;
            else if (point.x <= 10.0)
              points_front_far++;
          }
          if (point.y >= -0.6 && point.y <= 0.6)
            points_in_y_range++;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Point cloud bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]",
                  min_x, max_x, min_y, max_y);
      RCLCPP_INFO(this->get_logger(), "Detection zone: X=[%.2f, %.2f], Y=[%.2f, %.2f]",
                  minDetectionDistance_, maxDetectionDistance_,
                  -detectionWidth_ / 2.0, detectionWidth_ / 2.0);
      RCLCPP_INFO(this->get_logger(),
                  "Point distribution: front(0-1m)=%d, front(1-3m)=%d, front(3-10m)=%d, in_Y_range=%d, X>0=%d",
                  points_front_close, points_front_mid, points_front_far, points_in_y_range, points_x_positive);
      debug_log_count_++;
    }

    // Filter points within the detection zone (box filter)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : cloud_in->points)
    {
      // Detection zone: X (forward), Y (left/right)
      // X: from minDetectionDistance_ to maxDetectionDistance_ (in front of robot)
      // Y: from -detectionWidth_/2 to +detectionWidth_/2 (centered on robot)
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          point.x >= minDetectionDistance_ && point.x <= maxDetectionDistance_ &&
          point.y >= -detectionWidth_ / 2.0 && point.y <= detectionWidth_ / 2.0)
      {
        cloud_filtered->points.push_back(point);
      }
    }

    points_in_zone_ = cloud_filtered->points.size();

    // Perform clustering to identify distinct objects
    int cluster_count = 0;
    if (cloud_filtered->points.size() >= static_cast<size_t>(minClusterSize_))
    {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
      ece.setClusterTolerance(clusterTolerance_);
      ece.setMinClusterSize(minClusterSize_);
      ece.setMaxClusterSize(maxClusterSize_);
      ece.setSearchMethod(tree);
      ece.setInputCloud(cloud_filtered);
      ece.extract(cluster_indices);

      cluster_count = cluster_indices.size();
    }

    clusters_detected_ = cluster_count;

    // Determine if area is occupied
    // Occupied if: total points exceed threshold OR at least one valid cluster detected
    if (points_in_zone_ >= static_cast<size_t>(minPointsToOccupy_) || clusters_detected_ > 0)
    {
      area_is_clear_ = false;
    }
    else
    {
      area_is_clear_ = true;
    }

    detection_complete_ = true;

    // Publish filtered cloud for visualization/debugging
    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_filtered, *pc2_msg);
    pc2_msg->header.frame_id = msg->header.frame_id;
    pc2_msg->header.stamp = now();
    pc2_msg->is_dense = false;
    filtered_cloud_pub_->publish(*pc2_msg);
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const CheckAreaClear::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received CheckAreaClear goal request: %s", goal->check_area ? "true" : "false");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCheckAreaClear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCheckAreaClear> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{ std::bind(&CheckAreaClearActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCheckAreaClear> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing CheckAreaClear action");
    RCLCPP_INFO(this->get_logger(), "Waiting for point cloud data...");
    rclcpp::Rate loop_rate(20);  // 20 Hz check rate

    auto feedback = std::make_shared<CheckAreaClear::Feedback>();
    auto result = std::make_shared<CheckAreaClear::Result>();

    // Reset detection state
    detection_active_ = true;
    detection_complete_ = false;
    area_is_clear_ = true;
    points_in_zone_ = 0;
    clusters_detected_ = 0;
    debug_log_count_ = 0;

    int samples_collected = 0;
    int clear_samples = 0;
    int occupied_samples = 0;

    // Timeout tracking
    auto start_time = this->now();
    int timeout_warnings = 0;

    // Collect multiple samples for robustness
    while (samples_collected < numSamples_)
    {
      if (goal_handle->is_canceling())
      {
        detection_active_ = false;
        result->is_clear = true;  // Default to clear on cancel
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Check for timeout
      auto elapsed = (this->now() - start_time).seconds();
      if (elapsed > timeoutSeconds_)
      {
        detection_active_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Timeout: No point cloud data received after %.1f seconds. "
                    "Make sure the laser scan merger is running.",
                    timeoutSeconds_);

        // On timeout, default to clear (allow robot to proceed) but log warning
        result->is_clear = true;
        goal_handle->succeed(result);
        RCLCPP_WARN(this->get_logger(), "CheckAreaClear timed out - defaulting to is_clear=TRUE");
        return;
      }

      // Periodic warning if no data received
      if (!detection_complete_ && samples_collected == 0 && elapsed > 1.0 && timeout_warnings == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Still waiting for point cloud data...");
        timeout_warnings++;
      }

      if (detection_complete_)
      {
        samples_collected++;
        if (area_is_clear_)
        {
          clear_samples++;
        }
        else
        {
          occupied_samples++;
        }

        // Reset for next sample
        detection_complete_ = false;

        // Reset timeout after receiving data
        start_time = this->now();

        // Log progress
        RCLCPP_INFO(this->get_logger(), "Sample %d/%d: points=%zu, clusters=%d, clear=%s", samples_collected,
                    numSamples_, points_in_zone_, clusters_detected_, area_is_clear_ ? "YES" : "NO");

        feedback->status = "Sampling: " + std::to_string(samples_collected) + "/" + std::to_string(numSamples_);
        goal_handle->publish_feedback(feedback);
      }

      loop_rate.sleep();
    }

    detection_active_ = false;

    // Determine final result based on majority voting
    // If more than half of samples show occupied, area is occupied
    bool final_is_clear = (clear_samples > occupied_samples);

    result->is_clear = final_is_clear;

    if (rclcpp::ok())
    {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "CheckAreaClear completed: is_clear=%s (clear_samples=%d, occupied_samples=%d)",
                  final_is_clear ? "TRUE" : "FALSE", clear_samples, occupied_samples);
    }
  }

  void initialize_params()
  {
    // Point cloud topic parameter
    this->declare_parameter("cloudTopic", "/cloud_action");  // Default to /cloud_action

    // Detection zone parameters
    this->declare_parameter("minDetectionDistance", 0.0);  // Start detection from 0m (closer to robot)
    this->declare_parameter("maxDetectionDistance", 2.0);  // End detection at 2.0m
    this->declare_parameter("detectionWidth", 3.0);        // 3.0m wide detection zone (±1.5m)

    // Clustering parameters
    this->declare_parameter("minClusterSize", 5);
    this->declare_parameter("maxClusterSize", 500);
    this->declare_parameter("clusterTolerance", 0.15);

    // Detection threshold
    this->declare_parameter("minPointsToOccupy", 10);  // Minimum points to consider area occupied

    // Sampling parameters
    this->declare_parameter("numSamples", 5);  // Number of samples for robustness

    // Timeout parameter
    this->declare_parameter("timeoutSeconds", 5.0);  // Timeout waiting for point cloud data
  }

  void refresh_params()
  {
    this->get_parameter_or<float>("minDetectionDistance", minDetectionDistance_, 0.0);
    this->get_parameter_or<float>("maxDetectionDistance", maxDetectionDistance_, 2.0);
    this->get_parameter_or<float>("detectionWidth", detectionWidth_, 3.0);

    this->get_parameter_or<int>("minClusterSize", minClusterSize_, 5);
    this->get_parameter_or<int>("maxClusterSize", maxClusterSize_, 500);
    this->get_parameter_or<float>("clusterTolerance", clusterTolerance_, 0.15);

    this->get_parameter_or<int>("minPointsToOccupy", minPointsToOccupy_, 10);
    this->get_parameter_or<int>("numSamples", numSamples_, 5);
    this->get_parameter_or<float>("timeoutSeconds", timeoutSeconds_, 5.0);
  }

  // Parameters
  float minDetectionDistance_;
  float maxDetectionDistance_;
  float detectionWidth_;
  int minClusterSize_;
  int maxClusterSize_;
  float clusterTolerance_;
  int minPointsToOccupy_;
  int numSamples_;
  float timeoutSeconds_;

  // State variables
  bool detection_active_ = false;
  bool detection_complete_ = false;
  bool area_is_clear_ = true;
  size_t points_in_zone_ = 0;
  int clusters_detected_ = 0;
  int debug_log_count_ = 0;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
};
}  // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::CheckAreaClearActionServer)
