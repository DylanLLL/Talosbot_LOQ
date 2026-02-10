#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <chrono>
#include <cmath>
#include <vector>

#include "talosbot_custom_interfaces/action/check_area_clear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

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

    // Get the scan topic from parameter
    std::string scan_topic;
    this->get_parameter_or<std::string>("scanTopic", scan_topic, "/scan");
    RCLCPP_INFO(this->get_logger(), "Subscribing to laser scan topic: %s", scan_topic.c_str());

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&CheckAreaClearActionServer::scan_callback, this, std::placeholders::_1));

    this->action_server_ = rclcpp_action::create_server<CheckAreaClear>(
        this, "talosbot_check_area_clear", std::bind(&CheckAreaClearActionServer::handle_goal, this, _1, _2),
        std::bind(&CheckAreaClearActionServer::handle_cancel, this, _1),
        std::bind(&CheckAreaClearActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<CheckAreaClear>::SharedPtr action_server_;

  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!detection_active_)
    {
      return;
    }

    refresh_params();

    float front_angle_rad = frontAngleRange_ * M_PI / 180.0;
    float yaw_offset_rad = laserYawOffset_ * M_PI / 180.0;

    int rays_in_zone = 0;
    int total_front_rays = 0;

    // Track bounds of detected points for debugging (in robot frame)
    float det_min_x = std::numeric_limits<float>::max();
    float det_max_x = std::numeric_limits<float>::lowest();
    float det_min_y = std::numeric_limits<float>::max();
    float det_max_y = std::numeric_limits<float>::lowest();

    // Iterate through all laser rays
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      float range = msg->ranges[i];
      float laser_angle = msg->angle_min + i * msg->angle_increment;

      // Skip invalid readings
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max)
      {
        continue;
      }

      // Transform laser angle to robot frame by applying yaw offset
      // The LIDAR is rotated by laserYawOffset_ degrees in the URDF
      float robot_angle = laser_angle + yaw_offset_rad;

      // Normalize angle to [-pi, pi]
      while (robot_angle > M_PI)
        robot_angle -= 2.0 * M_PI;
      while (robot_angle < -M_PI)
        robot_angle += 2.0 * M_PI;

      // Only consider rays in the front angular range (relative to robot forward)
      if (std::abs(robot_angle) > front_angle_rad)
      {
        continue;
      }

      total_front_rays++;

      // Convert to cartesian in robot frame
      // robot_angle=0 is robot forward, X = forward, Y = left
      float x = range * std::cos(robot_angle);
      float y = range * std::sin(robot_angle);

      // Check if the hit point falls within the rectangular detection zone
      if (x >= minDetectionDistance_ && x <= maxDetectionDistance_ && y >= -detectionWidth_ / 2.0 &&
          y <= detectionWidth_ / 2.0)
      {
        rays_in_zone++;
        det_min_x = std::min(det_min_x, x);
        det_max_x = std::max(det_max_x, x);
        det_min_y = std::min(det_min_y, y);
        det_max_y = std::max(det_max_y, y);
      }
    }

    // Debug logging
    if (debug_log_count_ < 3)
    {
      // Analyze where front rays are actually hitting
      int rays_close = 0;     // range < minDetectionDistance_
      int rays_in_range = 0;  // range in [minDist, maxDist]
      int rays_far = 0;       // range > maxDetectionDistance_
      int rays_invalid = 0;
      float front_min_range = std::numeric_limits<float>::max();
      float front_max_range = std::numeric_limits<float>::lowest();

      for (size_t i = 0; i < msg->ranges.size(); i++)
      {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i * msg->angle_increment;

        if (std::abs(angle) > front_angle_rad)
        {
          continue;
        }

        if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max)
        {
          rays_invalid++;
          continue;
        }

        front_min_range = std::min(front_min_range, range);
        front_max_range = std::max(front_max_range, range);

        float x = range * std::cos(angle);
        if (x < minDetectionDistance_)
          rays_close++;
        else if (x > maxDetectionDistance_)
          rays_far++;
        else
          rays_in_range++;
      }

      RCLCPP_INFO(this->get_logger(), "LaserScan: %zu total rays, front_rays=%d, hits_in_zone=%d", msg->ranges.size(),
                  total_front_rays, rays_in_zone);
      RCLCPP_INFO(this->get_logger(),
                  "Front ray ranges: min=%.2f, max=%.2f | close(x<%.2f)=%d, in_range=%d, far(x>%.2f)=%d, invalid=%d",
                  front_min_range, front_max_range, minDetectionDistance_, rays_close, rays_in_range,
                  maxDetectionDistance_, rays_far, rays_invalid);
      RCLCPP_INFO(this->get_logger(), "Detection zone: X=[%.2f, %.2f], Y=[%.2f, %.2f], front_angle=±%.1f°",
                  minDetectionDistance_, maxDetectionDistance_, -detectionWidth_ / 2.0, detectionWidth_ / 2.0,
                  frontAngleRange_);
      if (rays_in_zone > 0)
      {
        RCLCPP_INFO(this->get_logger(), "Detected points bounds: X=[%.2f, %.2f], Y=[%.2f, %.2f]", det_min_x, det_max_x,
                    det_min_y, det_max_y);
      }
      debug_log_count_++;
    }

    hits_in_zone_ = rays_in_zone;

    // Determine if area is occupied
    // Occupied if number of rays hitting the zone >= threshold
    area_is_clear_ = (rays_in_zone < minRaysToOccupy_);

    detection_complete_ = true;
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
    RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
    rclcpp::Rate loop_rate(20);  // 20 Hz check rate

    auto feedback = std::make_shared<CheckAreaClear::Feedback>();
    auto result = std::make_shared<CheckAreaClear::Result>();

    // Reset detection state
    detection_active_ = true;
    detection_complete_ = false;
    area_is_clear_ = true;
    hits_in_zone_ = 0;
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
                    "Timeout: No laser scan data received after %.1f seconds. "
                    "Make sure the LIDAR is running.",
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
        RCLCPP_WARN(this->get_logger(), "Still waiting for laser scan data...");
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
        RCLCPP_INFO(this->get_logger(), "Sample %d/%d: hits=%d, clear=%s", samples_collected, numSamples_,
                    hits_in_zone_, area_is_clear_ ? "YES" : "NO");

        feedback->status = "Sampling: " + std::to_string(samples_collected) + "/" + std::to_string(numSamples_);
        goal_handle->publish_feedback(feedback);
      }

      loop_rate.sleep();
    }

    detection_active_ = false;

    // Determine final result based on majority voting
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
    // Laser scan topic parameter
    this->declare_parameter("scanTopic", "/scan");

    // Detection zone parameters (rectangular zone in front of robot)
    this->declare_parameter("minDetectionDistance", 0.65);  // Start detection beyond trolley bars (real: ~0.59m, sim: ~0.81m)
    this->declare_parameter("maxDetectionDistance", 2.5);   // End detection at 2.5m ahead
    this->declare_parameter("detectionWidth", 1.0);         // 1.0m wide detection zone (±0.5m from center)
    this->declare_parameter("frontAngleRange", 45.0);       // Only consider rays within ±45° from forward
    this->declare_parameter("laserYawOffset", 90.0);        // LIDAR yaw offset in degrees (from URDF rpy yaw=1.57)

    // Detection threshold
    this->declare_parameter("minRaysToOccupy", 3);  // Minimum laser rays hitting zone to consider occupied

    // Sampling parameters
    this->declare_parameter("numSamples", 5);

    // Timeout parameter
    this->declare_parameter("timeoutSeconds", 5.0);
  }

  void refresh_params()
  {
    this->get_parameter_or<float>("minDetectionDistance", minDetectionDistance_, 0.65);
    this->get_parameter_or<float>("maxDetectionDistance", maxDetectionDistance_, 2.5);
    this->get_parameter_or<float>("detectionWidth", detectionWidth_, 1.0);
    this->get_parameter_or<float>("frontAngleRange", frontAngleRange_, 45.0);
    this->get_parameter_or<float>("laserYawOffset", laserYawOffset_, 90.0);

    this->get_parameter_or<int>("minRaysToOccupy", minRaysToOccupy_, 3);
    this->get_parameter_or<int>("numSamples", numSamples_, 5);
    this->get_parameter_or<float>("timeoutSeconds", timeoutSeconds_, 5.0);
  }

  // Parameters
  float minDetectionDistance_;
  float maxDetectionDistance_;
  float detectionWidth_;
  float frontAngleRange_;  // Degrees: only consider rays within ±this angle from forward
  float laserYawOffset_;   // Degrees: LIDAR yaw rotation offset from robot forward
  int minRaysToOccupy_;
  int numSamples_;
  float timeoutSeconds_;

  // State variables
  bool detection_active_ = false;
  bool detection_complete_ = false;
  bool area_is_clear_ = true;
  int hits_in_zone_ = 0;
  int debug_log_count_ = 0;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};
}  // namespace talosbot_action

RCLCPP_COMPONENTS_REGISTER_NODE(talosbot_action::CheckAreaClearActionServer)
