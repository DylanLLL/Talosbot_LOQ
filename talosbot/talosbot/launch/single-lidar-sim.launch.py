from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  
  laser_converter_config = os.path.join(
      get_package_share_directory('talosbot'),
      'config',
      'laser-converter.yaml'
  )
  
  return LaunchDescription([
    Node(
      name='ros2_laser_scan_merger',
      package='ros2_laser_scan_merger',
      executable='ros2_laser_scan_merger',
      parameters=[laser_converter_config],
      output='screen',
      respawn=True,
      respawn_delay=5,
    )
  ])