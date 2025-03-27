#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Set the path for your nav2_params.yaml and map.yaml files
    default_params_file = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'my_nav2_project',
        'config',
        'nav2_params.yaml'
    )
    default_map_file = os.path.join(
        os.getenv('HOME'),
        'ros2_ws',
        'src',
        'my_nav2_project',
        'config',
        'map.yaml'
    )

    # Declare launch arguments for parameters and simulation time
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the Nav2 parameters file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )

    # Launch the TurtleBot3 Gazebo world (pre-built world with robot)
    gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'
        ],
        output='screen'
    )

    # Launch Nav2 bringup using your parameter and map files
    nav2_bringup = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
            f'params_file:={default_params_file}',
            f'map:={default_map_file}'
        ],
        output='screen'
    )

    # Launch your Battery Manager node (Python)
    battery_manager_node = Node(
        package='my_nav2_project',
        executable='battery_manager.py',
        name='battery_manager',
        output='screen'
    )

    # Launch your BehaviorTree Controller node (C++)
    bt_controller_node = Node(
        package='my_nav2_project',
        executable='bt_controller',
        name='bt_controller',
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        gazebo,
        nav2_bringup,
        battery_manager_node,
        bt_controller_node
    ])

