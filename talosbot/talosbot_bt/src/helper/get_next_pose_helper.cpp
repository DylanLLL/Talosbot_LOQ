#include "talosbot_bt/helper/get_next_pose_helper.hpp"

namespace talosbot_bt
{

BT::NodeStatus GetNextPoseHelper::tick()
{
  // return BT::NodeStatus::SUCCESS;
  httplib::Client cli("localhost", 5050);

  if (auto res = cli.Get("/api/tasks/get_tasks/talosbot1"))
  {
    if (res->status == httplib::StatusCode::OK_200)
    {
      nlohmann::json json_val = nlohmann::json::parse(res->body);
      std::string uuid = json_val["val"]["uuid"].get<std::string>();
      std::string locations = json_val["val"]["locations"].get<std::string>();
      std::string post_action = json_val["val"]["post_action"].get<std::string>();

      double pos_x = json_val["val"]["pos_x"].get<double>();
      double pos_y = json_val["val"]["pos_y"].get<double>();
      double quat_z = json_val["val"]["quat_z"].get<double>();
      double quat_w = json_val["val"]["quat_w"].get<double>();

      std::cout << "[pos_x]: " << pos_x << std::endl;
      std::cout << "[pos_y]: " << pos_y << std::endl;
      std::cout << "[quat_z]: " << quat_z << std::endl;
      std::cout << "[quat_w]: " << quat_w << std::endl;
      std::cout << "[uuid]: " << uuid << std::endl;
      std::cout << "[locations]: " << locations << std::endl;
      std::cout << "[post_action]: " << post_action << std::endl;

      setOutput("pos_x", pos_x);
      setOutput("pos_y", pos_y);
      setOutput("quat_z", quat_z);
      setOutput("quat_w", quat_w);
      setOutput("uuid", uuid);
      setOutput("locations", locations);
      setOutput("post_action", post_action);

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
