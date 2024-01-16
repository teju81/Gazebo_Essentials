#!/usr/bin/python3

import os
import launch
from launch import condition
from launch.actions.opaque_function import OpaqueFunction
from launch_ros.parameters_type import Parameters
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the robot description
    xacro_file_path = os.path.join(get_package_share_directory('volta_description'), 'urdf', 'volta.xacro')
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_description = {'robot_description': robot_description_config.toxml()}


    urdf = os.path.join(get_package_share_directory('volta_description'), 'urdf/volta_robot.urdf')

    # Get the world launch file
    world_choice_file = os.path.join(get_package_share_directory("volta_description"), "worlds", "neo_workshop.world")

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')



    params = {'robot_description': robot_description_config.toxml()}
    start_robot_state_publisher_cmd = Node(
                name='robot_state_publisher',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[params]
            )

    teleop =  Node(package='teleop_twist_keyboard',executable="teleop_twist_keyboard",
    output='screen',
    prefix = 'xterm -e',
    name='teleop')


    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_choice_file,
                'verbose': 'true',
            }.items()
        )

    spawn_entity = Node(
                name='urdf_spawner',
                package='gazebo_ros', 
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description',
                           '-entity', 'volta_robot',
                            '-x', '-7',
                           '-y', '-3',
                           '-Y', '1.5708',
                           '-package_to_model',
                           '-b'],
                output='screen'
            )         
    return LaunchDescription([gazebo, spawn_entity, start_robot_state_publisher_cmd, teleop])