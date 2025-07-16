#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='bobblebot',
            description='Name of the robot model'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ', 
                    PathJoinSubstitution([
                        FindPackageShare('bobble_description'),
                        'xacro',
                        'bobble_world.urdf.xacro'
                    ])
                ])
            }],
            output='screen'
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='bobble_spawner',
            arguments=[
                '-topic', 'robot_description',
                '-entity', LaunchConfiguration('model')
            ],
            output='screen'
        )
    ])