import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # Define the package name for easier reference
    package_name = 'ros2_slam_auto_navigation'

    # Declare the 'world_file' launch argument
    # This allows the user to specify the path to the Gazebo world file to be loaded
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(get_package_share_directory(package_name), 'worlds', 'simple.world'),
        description='Full path to the world file to load'
    )

    # Access the 'world_file' argument during runtime using LaunchConfiguration
    world_file = LaunchConfiguration('world_file')

    # Include the Robot State Publisher (RSP) launch file
    # This sets up the robot's description (URDF) and enables simulation time and ROS 2 control
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Specify the Twist Mux configuration file path
    # Twist Mux is used to manage velocity command inputs from different sources
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # Launch the Twist Mux node with the specified configuration file
    # Remap the output command velocity topic to match the robot's controller
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Specify the Gazebo parameters configuration file path
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Include the Gazebo simulator launch file
    # The 'world' argument specifies the world file to load
    # The 'extra_gazebo_args' argument includes the path to the Gazebo parameters file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': world_file,  # Use the world_file parameter defined earlier
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # Node to spawn the robot entity in Gazebo
    # The 'robot_description' topic contains the robot's URDF model
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Launch the diff_drive_controller for controlling the robot's differential drive system
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    # Launch the joint_state_broadcaster for publishing the robot's joint states
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    # Combine all the nodes and arguments into a LaunchDescription
    # This ensures all components are launched together
    return LaunchDescription([
        declare_world_file_cmd,  # Declare the world file argument for customization
        rsp,  # Robot State Publisher launch
        twist_mux,  # Twist Mux node for velocity command management
        gazebo,  # Gazebo simulator with world and parameters
        spawn_entity,  # Spawn the robot in the Gazebo world
        diff_drive_spawner,  # Start the differential drive controller
        joint_broad_spawner  # Start the joint state broadcaster
    ])
