#!/usr/bin/python3

from inspect import Arguments
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
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Get the robot description
    urdf_path = os.path.join(get_package_share_directory('volta_description'), 'urdf', 'volta.xacro')
    robot_description_config = xacro.process_file(
        urdf_path, mappings={"use_sim_time": "true"}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # # Get the path to volta control package
    # control_path =  os.path.join(get_package_share_directory('volta_control'))

    # # Get the control yaml file
    # volta_config=os.path.join(control_path, "config", "control.yaml")

    return LaunchDescription(
        [   
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'
            ),

            Node(
                name='robot_state_publisher',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description]
            ),

            Node(
                name='urdf_spawner',
                package='gazebo_ros', 
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description',
                           '-entity', 'volta_robot',
                           '-x', '-7',
                           '-y', '-3',
                           '-Y', '1.5708',
                           '-package_to_model',
                           '-b'
                            ],
                output='screen'
            )         

        ]
    )
