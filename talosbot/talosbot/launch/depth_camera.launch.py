import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        PathJoinSubstitution(
          [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
        )
      ),
      launch_arguments={
        # 'pointcloud.enable': 'true',
        'align_depth.enable': 'true',
        # 'enable_accel': 'true',
        # 'enable_gyro': 'true',
        # 'unite_imu_method': '1',
        # 'ordered_pc': 'true', 
        # 'initial_reset': 'true',
        # 'camera_namespace': 'talosbot1',
        # 'camera_name': 'front_camera',
        # 'global_time_enabled': 'false'
        'pointcloud.enable': 'true',
        'ordered_pc': 'true', 
        'initial_reset': 'true',
        'hole_filling_filter.enable': 'true',
        'hole_filling_filter': '3',
        # 'hole_filling_filter': '3'
        # 'enable_accel': 'true',
        # 'enable_gyro': 'true',
        # 'unite_imu_method': '1',
        # 'depth_width': '640',
        # 'depth_height':'480',
        # 'depth_fps': '6',
        # 'enable_color': 'false'
        
      }.items()
    ),
    # Node(
    #   name='imu_filter',
    #   package='imu_filter_madgwick',
    #   executable='imu_filter_madgwick_node',
    #   remappings=[
    #     ('/imu/data_raw', '/camera/imu')
    #   ],
    #   parameters=[{
    #     'use_mag':False, 
    #   }]
    # )
  ])