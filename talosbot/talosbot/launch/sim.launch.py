import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   gazebo_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('talosbot_gazebo'), 'launch'),
         '/gazebo.launch.py'])
   )
   # laser_scan_merger_launch = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(
   #       get_package_share_directory('ros2_laser_scan_merger'), 'launch'),
   #       '/merge_2_scan.launch.py'])
   # )
   laser_merger_config = os.path.join(
      get_package_share_directory('talosbot'),
      'config',
      'laser-merger-params.yaml'
   )

   laser_converter_config = os.path.join(
      get_package_share_directory('talosbot'),
      'config',
      'laser-converter.yaml'
   )

   laser_filter_config = os.path.join(
      get_package_share_directory('talosbot'), 
      'config', 
      'laser-filter.yaml'
   )
   # navigation_launch = IncludeLaunchDescription(
   #    PythonLaunchDescriptionSource([os.path.join(
   #       get_package_share_directory('talosbot_navigation'), 'launch'),
   #       '/navigation_sim.launch.py'])
   # )
   

   return LaunchDescription([
      gazebo_launch,
      Node(
         name='ros2_laser_scan_merger',
         package='ros2_laser_scan_merger',
         executable='ros2_laser_scan_merger',
         parameters=[laser_merger_config],
         output='screen',
         respawn=True,
         respawn_delay=2,
      ),
      Node(
         name='pointcloud_to_laserscan',
         package='pointcloud_to_laserscan',
         executable='pointcloud_to_laserscan_node',
         parameters=[laser_merger_config],
         respawn_delay=2,
      ),
      Node(
         name='ros2_laser_scan_merger',
         package='ros2_laser_scan_merger',
         executable='ros2_laser_scan_merger',
         parameters=[laser_converter_config],
         output='screen',
         respawn=True,
         respawn_delay=5,
      ),
      Node(
         package="laser_filters",
         executable="scan_to_scan_filter_chain",
         parameters=[laser_filter_config],
         respawn_delay=7,
      )
      # laser_scan_merger_launch,
      # navigation_launch
   ])