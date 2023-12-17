#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    play_rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '/dataset/hilti_dataset/exp21_outside_building']
    )
    
    exec_node = Node(
        package='eskf_lio',
        executable='eskf_lio_node',
        name='eskf_lio_node',
        prefix=['stdbuf -o L'],
        output='screen'
    )
    
    launch_description = LaunchDescription()
    launch_description.add_action(play_rosbag)
    launch_description.add_action(exec_node)
    return launch_description