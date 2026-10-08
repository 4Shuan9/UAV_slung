# UAV_slung

A unified ROS 2 + Gazebo/PX4 project for UAV slung-payload simulation, state estimation, and anti-swing control.  
This project is refactored from [rain_ws](https://github.com/RainbowSeeker/rain_ws.git) and [PX4-ROS2-Gazebo-Drone-Simulation-Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template.git), and has been adapted for PX4 v1.16.2 with a dedicated slung-load simulation world, vision-based payload tracking, swing-angle estimation, and multiple anti-swing flight modes.

<img width="800" height="450" alt="Image" src="/media/demo/QSLS.gif" />

## Key Features

* **Unified Project Structure:** Gazebo models/worlds and the ROS 2 control package are maintained in the same repository, without embedding a separate PX4 source tree.
* **PX4 v1.16.2 Compatibility:** The simulation setup is aligned with the PX4 v1.16.2 Gazebo workflow and ROS 2 DDS communication.
* **Enhanced Tether & Payload Dynamics:** The 1-meter tether is discretized into 20 articulated segments for more realistic slung-load motion.
* **Dedicated Slung-Load Environment:** Includes a custom UAV slung-payload world and project-specific Gazebo models.
* **Vision-Based Payload Tracking:** The downward camera pipeline publishes payload position through `/uav/cv/payload_pose`.
* **Swing-Angle Estimation:** Provides the state-estimation chain required by the anti-swing controller.
* **Anti-Swing Control:** Supports standard anti-swing control as well as Figure-8 and square trajectory experiments.
* **Integrated Simulation Launch:** `uav_slung_sim.launch.py` starts the main Gazebo / PX4 simulation chain and related vision process.
* **Resolved Sensor & Telemetry Issues:** Includes fixes for QGroundControl compass detection and PX4 ROS 2 telemetry topics such as `/fmu/out/vehicle_local_position`.

## System Requirements

| **Component** | **Version** |
| :--- | :--- |
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble |
| PX4 Autopilot | 1.16.2 |
| px4_msgs & px4_ros_com | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |
| Gazebo Harmonic | 8.x |
| ros_gz_bridge | gzharmonic |

## Project Structure

```text
UAV_slung/
├── docs/                       # Project notes and documentation
├── media/                      # Images / demos used by the documentation
├── ros2_ws/
│   └── src/
│       └── slung_ctrl/         # ROS 2 estimation and control package
├── sim/
│   ├── models/                 # Project-specific Gazebo models
│   └── worlds/                 # Project-specific Gazebo worlds
├── data_analysis/              # MATLAB / Python / CSV analysis tools
└── README.md
```

The external PX4 source tree is expected to remain at:

```text
~/PX4-Autopilot
```

and is **not** included inside this repository.

## Build

```bash
cd ~/UAV_slung/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash

colcon build --symlink-install

source install/setup.bash
```

After opening a new terminal, source the environment again before running ROS 2 commands:

```bash
cd ~/UAV_slung/ros2_ws

source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash
```

## Quick Start Guide

### 1. Start QGroundControl

Start QGroundControl normally before or after launching the simulation if flight-state monitoring is required.

### 2. Start Gazebo + PX4 Simulation

```bash
ros2 launch slung_ctrl uav_slung_sim.launch.py
```

This is the recommended simulation entry point.

### 3. Start Swing-Angle Estimation

Open a new terminal and source the workspace, then run:

```bash
ros2 launch slung_ctrl swing_angle.launch.py
```

### 4. Start Anti-Swing Control

For the standard anti-swing experiment:

```bash
ros2 launch slung_ctrl slung_antiswing.launch.py
```

### 5. Figure-8 Trajectory

```bash
ros2 launch slung_ctrl slung_antiswing_eight.launch.py
```

### 6. Square Trajectory

```bash
ros2 launch slung_ctrl slung_antiswing_square.launch.py
```

> Run only **one** flight-control launch at a time to avoid multiple nodes publishing conflicting PX4 Offboard setpoints.

## Useful Checks

Check PX4 → ROS 2 communication:

```bash
ros2 topic hz /fmu/out/vehicle_odometry
```

Check the vision payload-pose topic:

```bash
ros2 topic echo /uav/cv/payload_pose
```

List the active ROS 2 nodes:

```bash
ros2 node list
```

## PlotJuggler

For online plotting and experiment inspection:

```bash
ros2 run plotjuggler plotjuggler
```

## Main Launch Files

```text
uav_slung_sim.launch.py
swing_angle.launch.py
slung_antiswing.launch.py
slung_antiswing_eight.launch.py
slung_antiswing_square.launch.py
```

The older `slung_gazebo.launch.py` may still be kept for compatibility, but `uav_slung_sim.launch.py` is the recommended main simulation launcher.
