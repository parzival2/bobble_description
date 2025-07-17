#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def validate_world_file(context, *args, **kwargs):
    """Validate that the world file exists and return the correct path"""
    world_name = LaunchConfiguration('world').perform(context)
    package_share = get_package_share_directory('bobble_description')
    
    # If world_name is just a filename, look in our worlds directory
    if not world_name.startswith('/'):
        world_path = os.path.join(package_share, 'worlds', world_name)
    else:
        world_path = world_name
    
    # Check if file exists, fallback to empty_world.world if not
    if not os.path.exists(world_path):
        print(f"Warning: World file {world_path} not found, using empty_world.world")
        world_path = os.path.join(package_share, 'worlds', 'empty_world.world')
        
        if not os.path.exists(world_path):
            raise FileNotFoundError(f"Default world file not found: {world_path}")
    
    return world_path

def launch_gazebo_with_world(context, *args, **kwargs):
    """Launch Gazebo with validated world file"""
    world_path = validate_world_file(context)
    paused = LaunchConfiguration('paused').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    
    # gz sim bridge params
    bridge_params = os.path.join(
        get_package_share_directory('bobble_description'),
        'params',
        'ros_gz_bridge_params.yaml'
    )
    
    # Build Gazebo arguments
    gz_args = f"-r -s -v4 {world_path}"
    #if paused.lower() == 'true':
    # gz_args += " --pause"
    
    # Launch Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Command to launch ros_gz_bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    # Launch Gazebo GUI if requested
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )
    actions = [gazebo_server, bridge_cmd, gazebo_gui]
    
    return actions

def launch_robot_nodes(context, *args, **kwargs):
    """Launch robot nodes with configurable namespace support"""
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

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_namespace,
    )
    
    # Entity spawner with position parameters
    spawn_args = [
        '-topic', f'{topic_prefix}/robot_description',
        '-entity', model,
        '-x', x_pos,
        '-y', y_pos,
        '-z', z_pos
    ]
    
    if robot_namespace:
        spawn_args.extend(['-robot_namespace', robot_namespace])
    
    entity_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='bobble_spawner',
        arguments=spawn_args,
        output='screen'
    )
    
    return [robot_state_publisher, entity_spawner]

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description='Start Gazebo in paused state'
        ),
        
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo with GUI'
        ),
        
        DeclareLaunchArgument(
            'model',
            default_value='bobblebot',
            description='Name of the robot model'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value='empty_world.world',
            description='Gazebo world file (filename or full path)'
        ),
        
        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
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
        
        # Start Gazebo simulation
        OpaqueFunction(function=launch_gazebo_with_world),   
        
        # Robot state publisher with configurable namespace support
        OpaqueFunction(function=launch_robot_nodes)
    ])