from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      name='sllidar_lidar1',
      package='sllidar_ros2',
      executable='sllidar_node',
      namespace='lidar_1',
      parameters=[{
        'channel_type':'serial',
        'serial_port': '/dev/ttyUSB1', 
        'serial_baudrate': 1000000, 
        'frame_id': 'laser',
        'inverted': False, 
        'angle_compensate': True, 
        'scan_mode': 'DenseBoost'
      }]
    ),
    Node(
      name='sllidar_lidar2',
      package='sllidar_ros2',
      executable='sllidar_node',
      namespace='lidar_2',
      parameters=[{
        'channel_type':'serial',
        'serial_port': '/dev/ttyUSB0', 
        'serial_baudrate': 1000000, 
        'frame_id': 'laser',
        'inverted': False, 
        'angle_compensate': True, 
        'scan_mode': 'DenseBoost'
      }]
    )
  ])