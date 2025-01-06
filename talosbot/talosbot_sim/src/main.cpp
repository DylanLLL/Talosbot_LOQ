#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define MAX_COUNT 3
/*
  Stepper State
  0 -> At bottom
  10 -> Going Up
  20 -> At top
  30 -> Going Down
*/

class TalosbotSim : public rclcpp::Node
{
public:
  TalosbotSim() : Node("talosbot_sim")
  {
    battery_level_pub_ = this->create_publisher<std_msgs::msg::Float32>("talosbot1/battery_level", 10);
    locking_pin_status_pub_ = this->create_publisher<std_msgs::msg::String>("talosbot1/locking_pin_status", 10);

    locking_pin_command_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "talosbot1/locking_pin_command", 10,
        std::bind(&TalosbotSim::locking_pin_command_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(1000ms, std::bind(&TalosbotSim::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // auto message = std_msgs::msg::String();
    if ((locking_pin_state_ == 10) || (locking_pin_state_ == 30))
    {
      locking_pin_count_++;
    }
    if (locking_pin_count_ >= MAX_COUNT)
    {
      if (locking_pin_state_ == 10)
        locking_pin_state_ = 20;
      if (locking_pin_state_ == 30)
        locking_pin_state_ = 0;
    }

    auto battery_level_msg = std_msgs::msg::Float32();
    battery_level_msg.data = 53.24;

    auto locking_pin_status_msg = std_msgs::msg::String();
    locking_pin_status_msg.data = get_status_msg(locking_pin_state_);

    RCLCPP_INFO(this->get_logger(), "Publishing Battery Level: '%f'", battery_level_msg.data);
    RCLCPP_INFO(this->get_logger(), "Publishing Locking Pin Status: '%s'", locking_pin_status_msg.data.c_str());

    battery_level_pub_->publish(battery_level_msg);
    locking_pin_status_pub_->publish(locking_pin_status_msg);
  }

  void locking_pin_command_callback(const std_msgs::msg::Bool& msg)
  {
    if ((msg.data) && (locking_pin_state_ == 0))
    {
      locking_pin_state_ = 10;
      locking_pin_count_ = 0;
    }

    if ((!msg.data) && (locking_pin_state_ == 20))
    {
      locking_pin_state_ = 30;
      locking_pin_count_ = 0;
    }
  }

  std::string get_status_msg(int a)
  {
    if (a == 0)
      return "BOTTOM";
    if (a == 10)
      return "GOING UP";
    if (a == 20)
      return "TOP";
    if (a == 30)
      return "GOING DOWN";
    return "UNKNOWN";
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_level_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr locking_pin_status_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr locking_pin_command_sub_;

  int locking_pin_state_ = 0;
  int locking_pin_count_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalosbotSim>());
  rclcpp::shutdown();
  return 0;
}