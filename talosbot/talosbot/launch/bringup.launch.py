import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = False

    # joy_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    # )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("talosbot"), "config", "ekf.yaml"]
    )

    description_launch_path = PathJoinSubstitution(
        # [FindPackageShare('talosbot_description'), 'launch', 'description.launch.py']
        [FindPackageShare('talosbot_description'), 'launch', 'description.launch.py']
    )

    lidars_launch_path = PathJoinSubstitution(
        [FindPackageShare('talosbot'), 'launch', 'lidars-driver.launch.py']
    )

    depth_launch_path = PathJoinSubstitution(
        [FindPackageShare('talosbot'), 'launch', 'depth_camera.launch.py']
    )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidars_launch_path)
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depth_launch_path)
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'true',
                'rviz2': 'false'
            }.items()
        ),
    ])
