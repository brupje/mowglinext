# ROS2 Stack

The core ROS2 Jazzy workspace for MowgliNext — autonomous lawn mower with Nav2, SLAM Toolbox, behavior trees, and Fields2Cover coverage planning.

## Packages

| Package | Description |
|---------|-------------|
| `mowgli_interfaces` | Message, service, and action definitions |
| `mowgli_hardware` | Serial bridge to STM32 firmware (COBS + CRC-16) |
| `mowgli_description` | URDF/xacro robot model and meshes |
| `mowgli_localization` | Dual EKF (odom + map), GPS fusion, SLAM heading |
| `mowgli_nav2_plugins` | FTC and RotationShim controllers |
| `mowgli_coverage_planner` | Fields2Cover v2 autonomous mowing patterns |
| `mowgli_map` | Area management and costmap obstacle tracking |
| `mowgli_monitoring` | Diagnostics, MQTT bridge, telemetry |
| `mowgli_behavior` | BehaviorTree.CPP v4 main tree and nodes |
| `mowgli_simulation` | Gazebo Harmonic worlds and SDF model |
| `mowgli_bringup` | Launch files and configuration |
| `opennav_coverage` | Third-party Nav2 coverage server |

## Building

```bash
# Docker (recommended for ARM deployment)
docker build -t mowgli-ros2 --target runtime .

# Local (requires ROS2 Jazzy)
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --skip-keys "fields2cover" -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Launching

```bash
# Hardware
ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/mowgli

# Simulation
ros2 launch mowgli_bringup simulation.launch.py
```

## Documentation

Full documentation is on the [wiki](https://github.com/cedbossneo/mowglinext/wiki):

- [Architecture](https://github.com/cedbossneo/mowglinext/wiki/Architecture) — system design, data flow, TF tree
- [Configuration](https://github.com/cedbossneo/mowglinext/wiki/Configuration) — all YAML parameters
- [Simulation](https://github.com/cedbossneo/mowglinext/wiki/Simulation) — Gazebo Harmonic guide
- [Firmware](https://github.com/cedbossneo/mowglinext/wiki/Firmware) — STM32 COBS protocol
- [Behavior Trees](https://github.com/cedbossneo/mowglinext/wiki/Behavior-Trees) — BT nodes and control flow
