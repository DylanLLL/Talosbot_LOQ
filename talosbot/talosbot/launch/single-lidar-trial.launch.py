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
  ld_lidar_19_launch_file = PathJoinSubstitution(
        [FindPackageShare('ldlidar_stl_ros2'), 'launch', 'ld19.launch.py']
    )
  # config = os.path.join(
  #     get_package_share_directory('ros2_laser_scan_merger'),
  #     'config',
  #     'params.yaml'
  # )
  
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(ld_lidar_19_launch_file)
    ),
    # Node(
    #   name='sllidar_lidar1',
    #   package='sllidar_ros2',
    #   executable='sllidar_node',
    #   namespace='lidar_1',
    #   parameters=[{
    #     'channel_type':'serial',
    #     'serial_port': '/dev/ttyUSB1', 
    #     'serial_baudrate': 1000000, 
    #     'frame_id': 'laser',
    #     'inverted': False, 
    #     'angle_compensate': True, 
    #     'scan_mode': 'DenseBoost'
    #   }]
    # ),
    # Node(
    #   name='sllidar_lidar2',
    #   package='sllidar_ros2',
    #   executable='sllidar_node',
    #   namespace='lidar_2',
    #   parameters=[{
    #     'channel_type':'serial',
    #     'serial_port': '/dev/ttyUSB0', 
    #     'serial_baudrate': 1000000, 
    #     'frame_id': 'laser',
    #     'inverted': False, 
    #     'angle_compensate': True, 
    #     'scan_mode': 'DenseBoost'
    #   }]
    # ),
    # Node(
    #   name='ros2_laser_scan_merger',
    #   package='ros2_laser_scan_merger',
    #   executable='ros2_laser_scan_merger',
    #   parameters=[laser_merger_config],
    #   output='screen',
    #   respawn=True,
    #   respawn_delay=2,
    # ),
    # Node(
    #   name='pointcloud_to_laserscan',
    #   package='pointcloud_to_laserscan',
    #   executable='pointcloud_to_laserscan_node',
    #   parameters=[laser_merger_config],
    #   respawn_delay=2,
    # ),
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