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

    # Declaring a launch argument requires running three things - LaunchConfiguration, DeclareLaunchArgument and adding it to Launch Description

    world_choice_file = LaunchConfiguration('world')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory("volta_description"), "worlds", "neo_workshop.world"),
        description='Full path to the world file to be used by gazebo')

    declare_rviz_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(get_package_share_directory('volta_description'),"rviz_params","slam.rviz"),
        description='Full path to the rviz config file to be used by rviz')


    gazebo_launch_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('volta_description'), 'launch', 'gazebo_launch.py')
            ),
            launch_arguments={'world': world_choice_file}.items()
        )

    slam_params_file = os.path.join(get_package_share_directory('volta_description'),'config','mapper_params_online_async.yaml')
    slam_tool_box_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'), 'launch','online_async_launch.py')
        ),
        launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'true',
            }.items()
      )

    rviz_node = Node(
                name='rviz2',
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=['-d', rviz_config_file],
            )
    ld = LaunchDescription()
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_rviz_file_cmd)
    ld.add_action(gazebo_launch_node)
    ld.add_action(slam_tool_box_node)
    ld.add_action(rviz_node)


    return ld