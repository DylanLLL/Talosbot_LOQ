import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    # joy_launch_path = PathJoinSubstitution(
    #     [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    # )
    # Set the path to the Gazebo ROS package
    # pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    
    # # Set the path to this package.
    # pkg_share = FindPackageShare(package='talosbot_gazebo').find('talosbot_gazebo')
    
    # # Set the path to the world file
    # world_file_name = 'warehouse.world'
    # world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Set the path to the SDF model files.
    gazebo_models_path = PathJoinSubstitution(
        [FindPackageShare("talosbot_gazebo"), "models"]
    )
    os.environ["GAZEBO_MODEL_PATH"] = str(gazebo_models_path)

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("talosbot"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("talosbot_gazebo"), "worlds", "warehouse.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('talosbot_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "talosbot"]
        ),

        # Node(
        #     package='blibli_amr_gazebo',
        #     executable='command_timeout.py',
        #     name='command_timeout'
        # ),

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
                'publish_joints': 'false',
                'rviz2': 'false'
            }.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(joy_launch_path),
        # )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940