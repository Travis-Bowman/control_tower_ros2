#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    
    ibus_channel_arg = DeclareLaunchArgument('ibus_channel', default_value='/dev/ttyAMA0')
    
    control_tower_node = Node(
            package='control_tower_ros2',
            executable='control_tower_node',
            namespace="/",
            name='control_tower_node',
        )
    
    ibus_node = Node(
            package='ibus_reader',
            executable='ibus_reader',
            namespace="/",
            name='ibus_reader',
            parameters=[
                {'ibus_channel': LaunchConfiguration("ibus_channel")}
            ]
        )
    
    
    
    return LaunchDescription([
        ibus_channel_arg,
        control_tower_node,
        ibus_node,
    ])