# UAV_slung

ROS 2 + Gazebo/PX4 project for UAV slung-load simulation, vision-based swing estimation, delay compensation, and ADRC anti-swing control.

## Runtime architecture

The normal simulation/control chain is intentionally kept small:

```text
uav_slung_sim.launch.py
  ├─ Gazebo + PX4 + MicroXRCEAgent
  └─ gz_cam_cv.py
       ↓
swing_angle.launch.py
  ├─ truth bridge + swing_angle_truth
  ├─ swing_angle_cv
  ├─ vision_time_aligner
  └─ swing_angle_eso_fusion
       ↓
slung_antiswing.launch.py
  └─ ADRC / PX4 Offboard controller
       ↑
slung_antiswing_eight.launch.py or slung_antiswing_square.launch.py
  └─ optional trajectory generator publishing /uav/target_position
```

Important: `slung_antiswing_eight.launch.py` is **not** a second Offboard controller. It is a trajectory generator and is intended to run **together with** `slung_antiswing.launch.py` after the ADRC controller has taken off and entered CONTROL.

## System requirements

| Component | Version |
| --- | --- |
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble |
| PX4 Autopilot | 1.16.2 |
| px4_msgs / px4_ros_com | release/1.16 |
| Gazebo | Harmonic |
| ros_gz_bridge | Harmonic |

## Build

```bash
cd ~/UAV_slung/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For every new terminal:

```bash
cd ~/UAV_slung/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_ros2/install/setup.bash
source install/setup.bash
```

## Normal Figure-8 experiment startup order

### Terminal 1 — simulation

```bash
ros2 launch slung_ctrl uav_slung_sim.launch.py
```

### Terminal 2 — swing estimation

```bash
ros2 launch slung_ctrl swing_angle.launch.py
```

### Terminal 3 — ADRC / Offboard controller

```bash
ros2 launch slung_ctrl slung_antiswing.launch.py
```

Wait until takeoff is complete and the controller is in CONTROL.

### Terminal 4 — Figure-8 target generator

```bash
ros2 launch slung_ctrl slung_antiswing_eight.launch.py
```

Square trajectory is analogous:

```bash
ros2 launch slung_ctrl slung_antiswing_square.launch.py
```

Only one PX4 Offboard controller should run at a time. Trajectory-generator nodes are allowed to run alongside the ADRC controller.

## Fixed vs dynamic visual delay

The fusion estimator exposes two runtime modes:

```bash
# Fixed baseline: tau = 30 ms
ros2 param set /swing_angle_eso_fusion use_dynamic_delay false

# Dynamic delay: use the aligned visual sample age
ros2 param set /swing_angle_eso_fusion use_dynamic_delay true
```

Configuration is in `config/eso_params.yaml`.

The timing adapter publishes only:

```text
/uav/cv/swing_angle_aligned/world_frame/cartesian
```

Temporary timestamp-debug topics used during development were removed from the clean runtime graph.

## Quantitative experiment evaluator

`swing_error_eval` is an offline/experiment helper, not part of the controller. It measures:

- estimator vs Gazebo truth error;
- true swing RMS / P95 / maximum;
- horizontal trajectory tracking error;
- estimator-state age.

Example:

```bash
ros2 run slung_ctrl swing_error_eval --ros-args \
  -p experiment_name:=fixed_tau30 \
  -p duration_sec:=35.0 \
  -p output_csv:=/tmp/fixed_tau30.csv
```

By default the evaluator waits for ADRC CONTROL and the first external trajectory target, so it can be started before the Figure-8 node.

## Optional baseline estimators

The normal `swing_angle.launch.py` no longer starts every comparison estimator. If paper-comparison topics are needed, run in an extra terminal:

```bash
ros2 launch slung_ctrl swing_angle_baselines.launch.py
```

This starts the optional CV-compensation, standard ESO, and EKF estimators.

## Main runtime topics

```text
/uav/cv/payload_pose
/uav/cv/swing_angle/world_frame/cartesian
/uav/cv/swing_angle_aligned/world_frame/cartesian
/uav/truth/swing_angle/world_frame/cartesian
/uav/eso_fusion/swing_angle/world_frame/cartesian
/uav/eso_fusion/swing_rate/world_frame/cartesian
/uav/eso_fusion/disturbance/cartesian
/uav/eso_fusion/adrc_state
/uav/adrc/control_active
/uav/target_position
/uav/target_yaw
```

## Current known estimator issue / next development stage

The current fusion node still updates the LESO from the latest visual angle inside the high-rate IMU callback. Because vision is about 25 Hz while the IMU/Fusion loop is much faster, one visual sample can be reused across several prediction updates.

The next estimator change should therefore separate:

```text
new visual frame -> CORRECT once
high-rate inertial updates -> PREDICT only
```

Do not retune ADRC parameters before that structural multi-rate correction is implemented and revalidated.
