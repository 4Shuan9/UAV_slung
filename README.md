# UAV_slung

A Gazebo simulation environment for UAV slung-payload operations, with vision-based swing-angle estimation and anti-swing control.

Based on [rain_ws](https://github.com/RainbowSeeker/rain_ws.git) and [PX4-ROS2-Gazebo-Drone-Simulation-Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template.git), adapted for PX4 v1.16.2.

<img width="800" alt="UAV slung-load simulation" src="images/QSLS.gif" />

## Key Features

* **Basketball Court Environment:** Court markings provide visual references for manual flight and trajectory tracking.
* **Tether & Payload Dynamics:** A 1-meter tether is represented by 20 articulated segments and connected to a spherical payload.
* **Swing-Angle Estimation:** Vision measurements, traditional ESO, delay-compensated ESO with historical visual ZOH, two-state EKF and three-state EKF.
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

PX4 is expected at `~/PX4-Autopilot`, with its Gazebo submodule initialized to provide the stock `x500_mono_cam_down` model. Project-specific models are loaded from `sim/models/`; they do not need to be copied into PX4.

Camera detection requires `python3-gz-transport13`, `python3-gz-msgs10`, NumPy and OpenCV. For the 200 Hz PX4 acceleration configuration, see [the setup steps](docs/config/vehicle_local_position_200Hz.md).

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
ros2 launch slung_ctrl estimation.launch.py estimator:=eso_delay

# 3. Automatic takeoff and anti-swing hover; default takeoff height: 8 m
ros2 launch slung_ctrl control.launch.py estimator:=eso_delay

# 4. Optional: start the figure-eight trajectory after reaching hover
ros2 launch slung_ctrl trajectory.launch.py type:=eight

# Optional: PlotJuggler, when installed
ros2 run plotjuggler plotjuggler
```

Keep the simulation, estimation and control running, with one controller and one reference generator. Stopping the trajectory holds the last target; land and disarm in QGroundControl before shutdown.

For another PX4 directory, add `px4_dir:=/path/to/PX4-Autopilot` to the simulation command. Add `headless:=true` to run without Gazebo and camera windows.

PX4 runtime files are stored outside the project, by default in `~/.local/state/UAV_slung/px4/`. Keep `parameters.bson` to preserve settings; override the location with `runtime_dir:=/external/path`.

### Select Estimator and Trajectory

| `estimator` | Method | Controller |
| :--- | :--- | :---: |
| `eso_delay` | Historical visual ZOH ESO with delay compensation and prediction | ADRC / PD |
| `ekf3` | Three-state EKF with delayed-measurement replay | ADRC / PD |
| `eso` | Visual ZOH ESO without time alignment or delay compensation | ADRC / PD |
| `ekf2` | Original two-state EKF | PD |
| `vision_predictor` | Vision prediction baseline | Observation only |
| `none` | Vision and ground truth only | None |

`eso_delay` holds the previous visual sample over each past capture interval, integrates with time-aligned acceleration, then predicts to the current time without visual correction. It uses RK4 and no visual interpolation; out-of-order visual frames are rejected.

Use the same `estimator` in estimation and control. Add `controller:=pd` for PD control; EKF2 requires PD. Add `baselines:=true` to estimation to also run ESO, EKF2 and the vision predictor.

Trajectory `type` accepts `eight`, `square` or `waypoint`. The waypoint node accepts `geometry_msgs/msg/Point` on `/uav/reference/waypoint`.

Parameters are in `ros2_ws/src/slung_ctrl/config/`. Use `control_eso.yaml` for ESO / ESO_delay + ADRC, `control_ekf.yaml` for EKF3 + ADRC, and `control_pd.yaml` for PD. Set `takeoff_height` before launch.

The fixed baseline uses ESO_delay + ADRC; EKF3 + ADRC is the companion configuration. Parameter values and validation are summarized in [the parameter report](docs/config/parameter_selection.md). The figure-eight defaults to `eight_scale=8`, `omega=0.150` (maximum reference speed approximately 1.70 m/s).

## Main Topics

`<method>`: `eso`, `eso_delay`, `ekf2`, `ekf3` or `vision_predictor`. Rate is available for all except `vision_predictor`; disturbance for ESO, ESO_delay and EKF3; state for ESO, ESO_delay and EKF3.

| Topic | Message type | Purpose |
| :--- | :--- | :--- |
| `/uav/vision/payload_pixel` | `geometry_msgs/PointStamped` | Detected center, x/y in pixels |
| `/uav/swing/truth/angle` | `geometry_msgs/Vector3Stamped` | True swing angles |
| `/uav/swing/vision/angle` | `geometry_msgs/Vector3Stamped` | Visual swing angles |
| `/uav/swing/<method>/angle` | `geometry_msgs/Vector3Stamped` | Estimated swing angles |
| `/uav/swing/<method>/rate` | `geometry_msgs/Vector3Stamped` | Angular rates |
| `/uav/swing/<method>/disturbance` | `geometry_msgs/Vector3Stamped` | Estimated disturbance |
| `/uav/swing/<method>/state` | `slung_ctrl/SwingADRCState` | Combined angle/rate/disturbance feedback for ADRC |
| `/uav/reference/position` | `geometry_msgs/Point` | Target position, NED, m |
| `/uav/reference/yaw` | `std_msgs/Float32` | Target yaw, rad |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Position and velocity feedback |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 local position, velocity and acceleration |

Default rates: camera 25 Hz, ESO/ESO_delay/EKF3 output 200 Hz, control 50 Hz. The detector reads Gazebo Transport images directly.

## Coordinates and Units

* **Gazebo ENU:** x=East, y=North, z=Up. **PX4 NED:** x=North, y=East, z=Down; 8 m above the PX4 origin is z=-8 m. Their position origins may differ.
* **Swing messages:** vector.x=East angle, vector.y=North angle, in deg; rates in deg/s, disturbances in rad/s². These are payload swing angles, not UAV attitude.
* **Acceleration:** PX4 `ax`=North, `ay`=East; these are local velocity derivatives, not raw accelerometer measurements.
* **Geometry:** 1 m rope, suspension at the vehicle model origin; camera +0.10 m and base_link +0.24 m above it.
* **Time:** simulation uses `/clock` with `use_sim_time=true`. Vision stamps are image capture time; ESO state stamps are update time; ESO_delay/EKF3 state stamps are prediction time.
