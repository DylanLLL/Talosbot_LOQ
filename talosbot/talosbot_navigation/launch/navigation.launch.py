import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    keepout_launch_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_navigation'), 'launch', 'keepout_filter.launch.py']
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_navigation'), 'rviz', 'talosbot_navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_navigation'), 'maps', 'wsk_24_Jun.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_navigation'), 'config', 'navigation_mppi_keepout.yaml']
    )

    keepout_config_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_navigation'), 'config', 'keepout_params.yaml']
    )
    mask_map_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_navigation'), 'maps', 'wsk_24_Jun_filter.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='use_mask', 
            default_value='true',
            description='Enable mask usage to true'
        ),

        DeclareLaunchArgument(
            name='rviz2', 
            default_value='true',
            description='Run rviz2'
        ),

        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),
        
        DeclareLaunchArgument(
            name='mask_map', 
            default_value=mask_map_path,
            description='Keepout Filter map path'
        ),

        DeclareLaunchArgument(
            name='nav2_config', 
            default_value=nav2_config_path,
            description='Nav2 Config Path'
        ),

        DeclareLaunchArgument(
            name='keepout_config', 
            default_value=keepout_config_path,
            description='Keepout Config Path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(keepout_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': LaunchConfiguration("keepout_config"),
                'mask': LaunchConfiguration("mask_map")
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_mask"))
        ),
        
        TimerAction(
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_path),
                    launch_arguments={
                        'map': LaunchConfiguration("map"),
                        'use_sim_time': LaunchConfiguration("sim"),
                        'params_file': LaunchConfiguration("nav2_config")
                    }.items(),
                ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config_path],
                    condition=IfCondition(LaunchConfiguration("rviz2")),
                    parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
                ),
            ],
            period=3.0
        ),
    ])