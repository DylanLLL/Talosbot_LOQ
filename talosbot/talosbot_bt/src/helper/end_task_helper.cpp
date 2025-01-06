#include "talosbot_bt/helper/end_task_helper.hpp"

namespace talosbot_bt
{

BT::NodeStatus EndTaskHelper::tick()
{
  std::string tasks_uuid = getInput<std::string>("uuid").value();
  std::string tasks_status = getInput<std::string>("status").value();
  
  std::cout << tasks_uuid << std::endl;
  
  std::string request_url = "/api/tasks/end_tasks/" + tasks_uuid + "/" + tasks_status;
  
  std::cout << request_url << std::endl;
  httplib::Client cli("localhost", 5050);

  if (auto res = cli.Put(request_url))
  {
    if (res->status == httplib::StatusCode::OK_200)
    {
      return BT::NodeStatus::SUCCESS;
    }
    if (res->status == httplib::StatusCode::NotFound_404)
    {
      std::cout << "Failed" << std::endl;

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
