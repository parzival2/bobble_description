# ROS2 Migration Plan for bobble_description

## Overview

This document outlines the migration plan for the `bobble_description` package from ROS1 (Melodic/Noetic) to ROS2 (Humble/Iron/Rolling). The package provides URDF files and 3D meshes for the BobbleBot self-balancing robot for Gazebo simulation.

## Current Package Analysis

### Dependencies
- **Build system**: Catkin → Ament CMake
- **Gazebo plugins**: `hector_gazebo_plugins` → Gazebo ROS2 equivalents
- **Robot control**: `gazebo_ros_control` → `ros2_control` with Gazebo integration
- **Launch system**: XML launch files → Python launch files

### Package Structure
```
bobble_description/
├── package.xml          # Needs format update to format="3"
├── CMakeLists.txt       # Convert to ament_cmake
├── launch/              # Convert XML to Python launch files
├── xacro/               # Minimal changes required
└── meshes/              # No changes needed
```

## Migration Steps

### Phase 1: Package Configuration

#### 1.1 Update package.xml
- [x] Change package format to `format="3"`
- [x] Replace `<buildtool_depend>catkin</buildtool_depend>` with `<buildtool_depend>ament_cmake</buildtool_depend>`
- [x] Update dependencies:
  - Remove: `hector_gazebo_plugins`
  - Add: `gazebo_ros2_control`, `gazebo_ros_pkgs`, `robot_state_publisher`
- [x] Add exec dependencies for launch files: `launch`, `launch_ros`

#### 1.2 Update CMakeLists.txt
- [x] Change minimum CMake version to 3.8
- [x] Replace `find_package(catkin REQUIRED ...)` with `find_package(ament_cmake REQUIRED)`
- [x] Remove `catkin_package()` macro
- [x] Replace install commands with ament equivalents
- [x] Add `ament_package()` at the end

### Phase 2: Launch File Migration

#### 2.1 Convert bobble.launch
- [x] Create Python launch file bobble.launch.py
- [x] Add robot_state_publisher node with xacro processing  
- [x] Add spawn_entity node for Gazebo
- [x] Add model name launch argument
**Current (ROS1 XML):**
```xml
<launch>
  <param name="robot_description" command="$(find xacro)/xacro '$(find bobble_description)/xacro/bobble_world.urdf.xacro'" />
  <node name="bobble_spawner" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model bobblebot" respawn="false" output="screen" />
</launch>
```

**Target (ROS2 Python):**
```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='bobblebot'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', PathJoinSubstitution([FindPackageShare('bobble_description'), 'xacro', 'bobble_world.urdf.xacro'])])}]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', LaunchConfiguration('model')],
            output='screen'
        )
    ])
```

#### 2.2 Convert bobble_sim.launch
- [x] Create Python equivalent with Gazebo world loading
- [x] Add robot_state_publisher integration
- [x] Configure namespace support

### Phase 3: URDF/Xacro Updates

#### 3.1 Plugin Migration
- [x] Update gazebo_ros_control plugin to gazebo_ros2_control
- [x] Update hector_gazebo_ros_imu plugin to gazebo_ros_imu_sensor
- [x] Update camera plugin for ROS2 compatibility
**Current Gazebo plugins to replace:**

1. **gazebo_ros_control** → **gazebo_ros2_control**
```xml
<!-- OLD -->
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/bobble</robotNamespace>
  <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  <controlPeriod>0.002</controlPeriod>
</plugin>

<!-- NEW -->
<plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
  <parameters>$(find bobble_description)/config/ros2_controllers.yaml</parameters>
  <ros>
    <namespace>/bobble</namespace>
  </ros>
</plugin>
```

2. **hector_gazebo_ros_imu** → **gazebo_ros_imu_sensor**
```xml
<!-- OLD -->
<plugin name='gazebo_ros_imu' filename='libhector_gazebo_ros_imu.so'>
  <alwaysOn>true</alwaysOn>
  <updateRate>200.0</updateRate>
  <bodyName>bno055_imu_link</bodyName>
  <topicName>/imu_bosch/data_raw</topicName>
  <gaussianNoise>0.02</gaussianNoise>
</plugin>

<!-- NEW -->
<plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
  <bodyName>bno055_imu_link</bodyName>
  <topicName>/imu_bosch/data_raw</topicName>
  <updateRate>200.0</updateRate>
  <gaussianNoise>0.02</gaussianNoise>
  <ros>
    <namespace>/bobble</namespace>
  </ros>
</plugin>
```

#### 3.2 URDF Updates
- [x] Update xacro namespace declarations if needed
- [x] Verify mesh file paths (should remain unchanged)
- [x] Update plugin configurations in main URDF file

### Phase 4: Configuration Files

#### 4.1 Create ros2_controllers.yaml
- [x] Create ros2_controllers.yaml configuration file
- [x] Add controller manager configuration
- [x] Add diff_drive_controller configuration
- [x] Add joint_state_broadcaster configuration
```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz
    
    bobble_controller:
      type: diff_drive_controller/DiffDriveController
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

bobble_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    
    wheel_separation: 0.16
    wheel_radius: 0.0325
    
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal : [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

joint_state_broadcaster:
  ros__parameters: {}
```

### Phase 5: Build System Integration

#### 5.1 Workspace Setup
- [x] Use `colcon build` instead of `catkin_make`
- [x] Update build commands in CLAUDE.md
- [x] Verify package builds successfully (requires ROS2 environment)

#### 5.2 Testing
- [x] Update testing instructions for ROS2
- [x] Document launch file testing with `ros2 launch`
- [x] Document robot spawn verification in Gazebo
- [x] Document IMU data testing: `ros2 topic echo /bobble/imu_bosch/data_raw`
- [x] Document joint states testing: `ros2 topic echo /joint_states`

## Implementation Checklist

### Pre-Migration
- [ ] Backup current ROS1 package
- [ ] Set up ROS2 development environment
- [ ] Install required ROS2 packages: `gazebo_ros_pkgs`, `ros2_control`, `gazebo_ros2_control`

### Core Migration
- [x] Update package.xml to format="3"
- [x] Convert CMakeLists.txt to ament_cmake
- [x] Create Python launch files
- [x] Update URDF plugins
- [x] Create ros2_controllers.yaml configuration
- [x] Test build with `colcon build`

### Validation
- [ ] Launch robot in Gazebo: `ros2 launch bobble_description bobble_sim.launch.py`
- [ ] Verify all topics are publishing correctly
- [ ] Test robot control interface
- [ ] Update documentation and README

## Expected Challenges

1. **Plugin Compatibility**: Some Gazebo plugins may not have direct ROS2 equivalents
2. **Control Interface**: Migration from gazebo_ros_control to ros2_control requires configuration changes
3. **Launch File Complexity**: Python launch files are more verbose but offer greater flexibility
4. **Namespace Handling**: ROS2 has different namespace conventions

## Timeline Estimate

- **Phase 1-2**: 2-3 hours (Package config and launch files)
- **Phase 3**: 3-4 hours (URDF/plugin updates)
- **Phase 4-5**: 2-3 hours (Configuration and testing)
- **Total**: 7-10 hours

## References

- [ROS2 Migration Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Migration-Guide.html)
- [Gazebo ROS2 Integration](https://gazebosim.org/docs/latest/ros2_integration/)
- [ros2_control Documentation](https://control.ros.org/)
- [ROS2 Launch System](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-system.html)

## Post-Migration Notes

After successful migration:
1. Update companion repository `bobble_controllers` for ROS2 compatibility
2. Update all documentation and tutorials
3. Consider creating migration scripts for similar packages
4. Test integration with ROS2 navigation stack if applicable