import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   marunda_gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('talosbot_gazebo'), 'launch'),
         '/marunda.launch.py'])
   )
   laser_scan_merger_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros2_laser_scan_merger'), 'launch'),
         '/merge_2_scan.launch.py'])
   )
   navigation_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('talosbot_navigation'), 'launch'),
         '/navigation_sim.launch.py'])
   )

   return LaunchDescription([
      marunda_gazebo_launch,
      laser_scan_merger_launch,
      # navigation_launch
   ])