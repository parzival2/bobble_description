#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def validate_urdf_file(context, *args, **kwargs):
    """Validate that the URDF file exists"""
    try:
        package_share = get_package_share_directory('bobble_description')
        urdf_path = os.path.join(package_share, 'xacro', 'bobble_world.urdf.xacro')
        
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        
        print(f"Using URDF file: {urdf_path}")
        return urdf_path
    except Exception as e:
        print(f"Error validating URDF file: {e}")
        raise

def launch_robot_nodes(context, *args, **kwargs):
    """Launch robot nodes with error handling"""
    validate_urdf_file(context)
    
    use_namespace = LaunchConfiguration('use_namespace').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    model = LaunchConfiguration('model').perform(context)
    x_pos = LaunchConfiguration('x_pos').perform(context)
    y_pos = LaunchConfiguration('y_pos').perform(context)
    z_pos = LaunchConfiguration('z_pos').perform(context)
    
    # Determine namespace settings
    robot_namespace = namespace if use_namespace.lower() == 'true' else None
    topic_prefix = f'/{namespace}' if robot_namespace else ''
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
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
    )
    
    # Entity spawner (for existing Gazebo instance) with position parameters
    spawn_args = [
        '-topic', f'{topic_prefix}/robot_description' if robot_namespace else 'robot_description',
        '-entity', model,
        '-x', x_pos,
        '-y', y_pos,
        '-z', z_pos
    ]
    
    if robot_namespace:
        spawn_args.extend(['-robot_namespace', robot_namespace])
    
    entity_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='bobble_spawner',
        arguments=spawn_args,
        output='screen'
    )
    
    return [robot_state_publisher, entity_spawner]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='bobblebot',
            description='Name of the robot model'
        ),
        
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to use namespace for the robot'
        ),
        
        DeclareLaunchArgument(
            'namespace',
            default_value='bobble',
            description='Robot namespace'
        ),
        
        DeclareLaunchArgument(
            'x_pos',
            default_value='0.0',
            description='X position of robot spawn point'
        ),
        
        DeclareLaunchArgument(
            'y_pos',
            default_value='0.0',
            description='Y position of robot spawn point'
        ),
        
        DeclareLaunchArgument(
            'z_pos',
            default_value='0.035',
            description='Z position of robot spawn point (wheel radius + small margin)'
        ),
        
        # Launch robot nodes with error handling
        OpaqueFunction(function=launch_robot_nodes)
    ])