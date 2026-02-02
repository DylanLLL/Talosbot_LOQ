import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_locking_pin_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_go_under_trolley_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_move_base_linear_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_wait_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_go_middle_front_door_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_move_base_angular_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_relocalize_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_set_tolerance_action_server',
            output='screen',
        ),
        Node(
            name='talosbot_action',
            package='talosbot_action',
            executable='talosbot_check_area_clear_action_server',
            output='screen',
        )
    ])
