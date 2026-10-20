# UAV_slung

A Gazebo simulation environment for UAV slung-payload operations, with vision-based swing-angle estimation and anti-swing control.

Based on [rain_ws](https://github.com/RainbowSeeker/rain_ws.git) and [PX4-ROS2-Gazebo-Drone-Simulation-Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template.git), adapted for PX4 v1.16.2.

<img width="800" alt="UAV slung-load simulation" src="images/QSLS.gif" />

## Key Features

* **Basketball Court Environment:** Court markings provide visual references for manual flight and trajectory tracking.
* **Tether & Payload Dynamics:** A 1-meter tether is represented by 20 articulated segments and connected to a spherical payload.
* **Swing-Angle Estimation:** Vision measurements, traditional ESO, delay-compensated ESO fusion, two-state EKF and three-state EKF.
* **Anti-Swing Control:** ADRC and PD control with figure-eight, square and waypoint references.

## System Requirements

| Component | Requirement |
| :--- | :--- |
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4 Autopilot | 1.16.2 |
| px4_msgs | release/1.16 |
| Micro-XRCE-DDS-Agent | 3.0.1 |
| Gazebo | Harmonic |
| ros_gz_bridge | gzharmonic |

PX4 is expected at `~/PX4-Autopilot`, including the `x500_mono_cam_down` model. Project models are loaded from `sim/models/`; they do not need to be copied into PX4.

## Quick Start Guide

### Build

```bash
cd ~/UAV_slung/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install
source install/setup.bash
```

`~/ws_ros2` is the dependency workspace containing px4_msgs. Adjust it to your installation. Load the following environment in each new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source ~/UAV_slung/ros2_ws/install/setup.bash
```

### Run

Open QGroundControl, then run each command in a separate terminal:

```bash
# 1. Basketball-court simulation, PX4, DDS agent and camera detection
ros2 launch slung_ctrl sim.launch.py

# 2. Vision, ground truth and swing-angle estimation
ros2 launch slung_ctrl estimation.launch.py estimator:=eso_fusion

# 3. Automatic takeoff and anti-swing hover; default takeoff height: 10 m
ros2 launch slung_ctrl control.launch.py estimator:=eso_fusion

# 4. Optional: start the figure-eight trajectory after reaching hover
ros2 launch slung_ctrl trajectory.launch.py type:=eight

# Optional: PlotJuggler, when installed
ros2 run plotjuggler plotjuggler
```

Keep the simulation, estimation and control terminals running. Run only one controller and one reference generator at a time. Stopping the reference generator holds its last target; it does not land the UAV. Use QGroundControl to land and disarm before shutting down the simulation.

For another PX4 directory, add `px4_dir:=/path/to/PX4-Autopilot` to the simulation command. Add `headless:=true` to run without Gazebo and camera windows.

PX4 parameters and runtime files are stored outside the project in `~/.local/state/UAV_slung/px4/` (`$XDG_STATE_HOME/UAV_slung/px4/` when set); flight logs are under its `log/` directory. Override with `runtime_dir:=/external/path` if needed. Keep `parameters.bson` to preserve PX4 settings. Logs can be removed after the simulation stops.

### Select Estimator and Trajectory

| `estimator` | Method | Controller |
| :--- | :--- | :---: |
| `eso_fusion` | Historical ESO with prediction to the current time | ADRC / PD |
| `ekf3` | Three-state EKF with delayed-measurement replay | ADRC / PD |
| `eso` | Traditional ESO | PD |
| `ekf2` | Original two-state EKF | PD |
| `vision_predictor` | Vision prediction baseline | Observation only |
| `none` | Vision and ground truth only | None |

Use the same `estimator` in estimation and control. For EKF3, replace `eso_fusion` with `ekf3` in both commands. For traditional ESO or EKF2, add `controller:=pd` to the control command. Add `baselines:=true` to estimation to also run ESO, EKF2 and the vision predictor.

Trajectory `type` accepts `eight`, `square` or `waypoint`. The waypoint node accepts `geometry_msgs/msg/Point` on `/uav/reference/waypoint`.

Configuration files are under `ros2_ws/src/slung_ctrl/config/`: estimator parameters in `<estimator>.yaml`, controller parameters in `control_eso.yaml`, `control_ekf.yaml` or `control_pd.yaml`, and trajectory parameters in `eight.yaml` or `square.yaml`. Set `takeoff_height` in the selected controller configuration before launch.

## Topic Interfaces

`<method>` denotes `eso`, `eso_fusion`, `ekf2`, `ekf3` or `vision_predictor`.

| Topic | Message Type | Purpose |
| :--- | :--- | :--- |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo simulation time |
| `/model/x500_mono_cam_down/pose` | `geometry_msgs/PoseStamped` | UAV pose in the Gazebo world |
| `/model/payload_ball/pose` | `geometry_msgs/PoseStamped` | Payload pose in the Gazebo world |
| `/uav/vision/payload_pixel` | `geometry_msgs/PointStamped` | Detected payload center, x/y in pixels |
| `/uav/swing/vision/angle` | `geometry_msgs/Vector3Stamped` | Raw vision projection angles |
| `/uav/swing/truth/angle` | `geometry_msgs/Vector3Stamped` | Ground-truth projection angles |
| `/uav/swing/<method>/angle` | `geometry_msgs/Vector3Stamped` | Estimated projection angles |
| `/uav/swing/<method>/rate` | `geometry_msgs/Vector3Stamped` | Angular rates; ESO, ESO fusion, EKF2 and EKF3 |
| `/uav/swing/<method>/disturbance` | `geometry_msgs/Vector3Stamped` | Disturbance estimates; ESO, ESO fusion and EKF3 |
| `/uav/swing/eso_fusion/state` | `slung_ctrl/SwingADRCState` | Synchronized ESO feedback for ADRC |
| `/uav/swing/ekf3/state` | `slung_ctrl/SwingADRCState` | Synchronized EKF3 feedback for ADRC |
| `/uav/swing/eso_fusion/diagnostics` | `std_msgs/Float64MultiArray` | Input/vision timing and frame counters |
| `/uav/swing/ekf3/diagnostics` | `std_msgs/Float64MultiArray` | Input/vision timing, NIS and event count |
| `/uav/swing/ekf3/covariance` | `std_msgs/Float64MultiArray` | East 3×3 covariance, then North 3×3; row-major |
| `/uav/reference/position` | `geometry_msgs/Point` | Position reference, local NED, m |
| `/uav/reference/yaw` | `std_msgs/Float32` | PX4 yaw reference, rad |
| `/uav/reference/waypoint` | `geometry_msgs/Point` | External waypoint input, local NED, m |
| `/uav/control/active` | `std_msgs/Bool` | ADRC has entered the CONTROL stage |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 local position, velocity and acceleration |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 odometry for control |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Offboard control mode |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Flight-control setpoint |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Flight-mode and arm/disarm requests |

The camera detector reads the Gazebo Transport image topic `/world/UAV_slung_world/model/x500_mono_cam_down/link/camera_link/sensor/imager/image`; this is not a ROS image topic.

Vision, truth and vision prediction also provide `angle_body`, `polar` and `polar_body` under their respective `/uav/swing/<source>/` prefixes. `polar` contains tilt angle / azimuth / rope length, in deg / deg / m.

## Coordinate Frames and Units

* **Gazebo world / ENU:** x=East, y=North, z=Up. World swing-angle messages use `frame_id=world`.
* **PX4 local NED:** x=North, y=East, z=Down. A target 10 m above the PX4 local origin has z=-10 m. PX4 and Gazebo position origins may differ.
* **Swing feedback:** vector.x=East projection angle, vector.y=North projection angle, in deg. Angular rates are deg/s; disturbances are rad/s². These are two pendulum projection angles, not UAV Euler angles.
* **Acceleration:** `VehicleLocalPosition.ax` is North acceleration and `ay` is East acceleration. These are local-frame velocity derivatives, not raw body-frame accelerometer specific force.
* **Body-angle outputs:** `frame_id=uav_base_link`; x/y are body-axis projection angles and must not be read as fixed East/North angles. Vision/truth/predictor angle messages retain rope length in vector.z; estimator angle messages do not provide rope length.
* **ADRC state:** `SwingADRCState` explicitly labels East/North angle, rate and disturbance fields. ESO fusion uses a linear-model residual; EKF3 uses a nonlinear-model residual. Use the matching controller configuration.
* **Time:** simulation nodes default to `use_sim_time=true`. Vision timestamps represent acquisition time; ESO fusion/EKF3 state timestamps represent prediction/publication time. Hardware deployment requires `use_sim_time:=false` and aligned camera/PX4 timestamps.
