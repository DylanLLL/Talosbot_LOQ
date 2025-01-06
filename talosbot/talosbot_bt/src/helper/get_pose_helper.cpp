#include "talosbot_bt/helper/get_pose_helper.hpp"
#include <rclcpp/rclcpp.hpp>
namespace talosbot_bt
{

BT::NodeStatus GetPoseHelper::tick()
{
  RCLCPP_INFO(rclcpp::get_logger("BT_GET_POSE_HELPER"), "Start BT Get Pose Helper");

  std::string location_name = getInput<std::string>("locations").value();
  std::string request_endpoint = "/api/positions/locations/" + location_name;
  httplib::Client cli("localhost", 5050);
  if (auto res = cli.Get(request_endpoint.c_str()))
  {
    if (res->status == httplib::StatusCode::OK_200)
    {
      nlohmann::json json_val = nlohmann::json::parse(res->body);
      double pos_x = json_val["val"]["pos_x"].get<double>();
      double pos_y = json_val["val"]["pos_y"].get<double>();
      double quat_z = json_val["val"]["quat_z"].get<double>();
      double quat_w = json_val["val"]["quat_w"].get<double>();

      std::cout << "[pos_x]: " << pos_x << std::endl;
      std::cout << "[pos_y]: " << pos_y << std::endl;
      std::cout << "[quat_z]: " << quat_z << std::endl;
      std::cout << "[quat_w]: " << quat_w << std::endl;

      setOutput("pos_x", pos_x);
      setOutput("pos_y", pos_y);
      setOutput("quat_z", quat_z);
      setOutput("quat_w", quat_w);

      return BT::NodeStatus::SUCCESS;
    }
    if (res->status == httplib::StatusCode::NotFound_404)
    {
      std::cout << "No Task" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    auto err = res.error();
    std::cout << err << std::endl;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace talosbot_bt
