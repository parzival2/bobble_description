# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 package (`bobble_description`) that provides URDF/xacro files, 3D meshes, Gazebo plugins, and controller configuration for the BobbleBot — a self-balancing two-wheeled robot designed for Gazebo simulation. The package follows standard ROS2 ament_cmake conventions.

## Build Commands

```bash
# Build the package (run from workspace root ~/Projects/ros_ws)
colcon build --packages-select bobble_description

# Build entire workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Launch Commands

```bash
# Launch full simulation (Gazebo + robot + controllers)
ros2 launch bobble_description bobble_sim.launch.py

# Launch with custom parameters
ros2 launch bobble_description bobble_sim.launch.py gui:=true paused:=false

# Spawn into existing Gazebo instance
ros2 launch bobble_description bobble.launch.py
```

### Launch Parameters (`bobble_sim.launch.py`)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `paused` | `true` | Start Gazebo paused |
| `gui` | `true` | Show Gazebo GUI |
| `model` | `bobblebot` | Robot model name |
| `world` | `empty_world.world` | World file |
| `use_namespace` | `true` | Enable namespace |
| `namespace` | `bobble` | Robot namespace |
| `x_pos` / `y_pos` / `z_pos` | `0.0` / `0.0` / `0.035` | Spawn position |

## Architecture

### URDF Structure
The robot description is modular, built using xacro macros:

- **Main assembly**: `xacro/bobble.urdf.xacro` — includes all components
- **Gazebo plugins**: `xacro/bobble.gazebo` — ros2_control hardware interface + IMU sensor
- **Component files** (each in `xacro/`):
  - `bobble_chassis.urdf.xacro` — main chassis (mass: 2.043 kg, collision box)
  - `bobble_chassis2.urdf.xacro` — secondary chassis (orange visual only)
  - `bobble_battery.urdf.xacro` — battery module (visual only)
  - `bobble_electronics.urdf.xacro` — electronics housing (visual only)
  - `bobble_realsense.urdf.xacro` — RealSense camera mount (visual only)
  - `left_wheel.urdf.xacro` / `right_wheel.urdf.xacro` — wheel assemblies (mass: 0.084 kg each, radius: 0.0275 m)
  - `bno055_imu.urdf.xacro` — BNO055 IMU sensor (100 Hz, publishes to `/bobble/imu`)
  - `camera.urdf.xacro` — camera component (currently commented out)

### Robot Link Tree
```
bobble_chassis_link (base)
├── bobble_battery_link (fixed, visual only)
├── bobble_electronics_link (fixed, visual only)
├── bobble_realsense_link (fixed, visual only)
├── bobble_chassis2_link (fixed, visual only)
├── bno055_imu_link (fixed, IMU sensor)
├── left_wheel_link (continuous joint: left_wheel_hinge)
└── right_wheel_link (continuous joint: right_wheel_hinge)
```

### Controller Configuration (`config/ros2_controllers.yaml`)

- **Controller manager update rate**: 100 Hz
- **Diff drive controller** (`bobble_controller`):
  - Wheel separation: 0.16 m, wheel radius: 0.0325 m
  - Velocity limits: ±1.0 m/s linear, ±1.0 rad/s angular
  - Acceleration limits: 1.0 m/s², 1.0 rad/s²
  - `open_loop: false` — uses wheel encoder feedback
  - `enable_odom_tf: false` — see "Odometry & TF" below
  - Publishes raw wheel odometry to `/bobble_controller/odom`
- **Joint state broadcaster**: Publishes joint states

### Odometry & TF Architecture

**`enable_odom_tf: false` is intentional.** The diff_drive_controller does NOT publish the `odom` → `bobble_chassis_link` TF. Instead, the **EKF node** in the companion `bobble_localization` package publishes this transform after fusing:
- Wheel odometry (linear velocity) from `/bobble_controller/odom`
- IMU data (orientation + angular velocity) from `/bobble/imu`

This gives a more accurate pose estimate than raw wheel odometry alone, which is critical for a self-balancing robot. If both published the same TF, you'd get conflicting transforms causing flickering and degraded localization.

### Key Topics
| Topic | Type | Source |
|-------|------|--------|
| `/bobble/cmd_vel` | `geometry_msgs/TwistStamped` | Input to diff drive controller |
| `/bobble_controller/odom` | `nav_msgs/Odometry` | Raw wheel odometry (diff drive) |
| `/bobble/odom/filtered` | `nav_msgs/Odometry` | Fused odometry (EKF in bobble_localization) |
| `/bobble/imu` | `sensor_msgs/Imu` | IMU data from Gazebo plugin |

### Gazebo Integration
- **Hardware interface**: `gz_ros2_control/GazeboSimSystem` — velocity command interface for both wheels
- **IMU plugin**: Gazebo IMU sensor at 100 Hz on `bno055_imu_link`
- **ROS-Gazebo bridge** (`params/ros_gz_bridge_params.yaml`): Bridges `clock` and `/bobble/imu` topics

### Mesh Files
3D STL meshes in `meshes/` — all scaled by 0.001 (mm to m conversion):
- `chassis_grey.stl`, `chassis_orange.stl`, `battery.stl`, `electronics.stl`, `realsense.stl`, `wheel.stl`

### Dependencies
- `gazebo_ros2_control` — Gazebo hardware interface
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image` — Gazebo-ROS2 integration
- `robot_state_publisher` — publishes robot state from URDF
- `launch`, `launch_ros` — launch system

### Related Packages
- **`bobble_localization`** (in same workspace) — EKF-based sensor fusion, publishes the odom TF
- **[bobble_controllers](https://github.com/super-owesome/bobble_controllers)** — control algorithms and simulation setup

IMPORTANT: Once you set yourself some todos, you should complete them without stopping,
*unless*
 you have some particular
*reason*
 to stop (e.g. you need to ask a question, you need feedback / input, etc.) If you do stop before finishing all todos, you MUST state the reason why you're stopping. We'll call this the "no-stop" rule. If you stop before finishing todos, and don't give any reason why you've stopped, I'll be very disappointed, and I'll ask you why you didn't follow the "no-stop rule."
