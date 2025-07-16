#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('bobble_description'),
        'worlds',
        'empty_world.world'
    )
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description='Start Gazebo in paused state'
        ),
        
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start Gazebo with GUI'
        ),
        
        DeclareLaunchArgument(
            'model',
            default_value='bobblebot',
            description='Name of the robot model'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value='empty_sky.world',
            description='Gazebo world file'
        ),
        
        # Start Gazebo with empty_sky world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
        ),   
        
        # Robot state publisher with namespace support
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='bobble',
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
        
        # Spawn entity in Gazebo with namespace
        Node(
            package='ros_gz_sim',
            executable='create',
            name='bobble_spawner',
            arguments=[
                '-topic', '/bobble/robot_description',
                '-entity', LaunchConfiguration('model'),
                '-robot_namespace', 'bobble'
            ],
            output='screen'
        )
    ])