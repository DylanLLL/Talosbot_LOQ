#include "talosbot_bt/helper/print_pose_helper.hpp"

namespace talosbot_bt
{

BT::NodeStatus PrintPoseHelper::tick()
{
  // return BT::NodeStatus::SUCCESS;
  auto val1 = getInput<double>("pos_x");
  auto val2 = getInput<double>("pos_y");
  auto val3 = getInput<double>("quat_z");
  auto val4 = getInput<double>("quat_w");

  printf("pos_x: %f | pos_y: %f | quat_z: %f | quat_w: %f\n", val1.value(), val2.value(), val3.value(), val4.value());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace talosbot_bt
