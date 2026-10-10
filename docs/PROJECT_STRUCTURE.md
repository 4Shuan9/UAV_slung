# Project structure notes

```text
UAV_slung/
├── README.md
├── docs/
├── media/
├── sim/
│   ├── models/
│   └── worlds/
└── ros2_ws/
    └── src/
        └── slung_ctrl/
            ├── config/
            ├── launch/
            ├── msg/
            ├── scripts/
            └── src/
```

## Runtime groups

### Simulation

- `uav_slung_sim.launch.py` — preferred simulation entry point.
- `slung_gazebo.launch.py` — implementation launch used by the wrapper above.
- `gz_cam_cv.py` — Gazebo camera decoding + payload detection.

### Core estimation

- `swing_angle.launch.py` — runtime estimator chain.
- `swing_angle_truth.cpp` — Gazebo truth.
- `swing_angle_cv.cpp` — raw visual swing angle.
- `vision_time_aligner.py` — Gazebo sim-time to system-time timestamp alignment.
- `swing_angle_eso_fusion.cpp` — LESO fusion + fixed/dynamic delay state prediction.

### Optional estimator baselines

- `swing_angle_baselines.launch.py`
- `swing_angle_cv_comp.cpp`
- `swing_angle_eso.cpp`
- `swing_angle_ekf.cpp`

### Flight control

- `slung_antiswing.launch.py` / `slung_antiswing_ADRC.cpp` — PX4 Offboard controller, takeoff, position loop, ADRC anti-swing.
- `slung_antiswing_eight.launch.py` / `slung_antiswing_eight.cpp` — Figure-8 target generator; it does not replace the ADRC controller.
- `slung_antiswing_square.launch.py` / `slung_antiswing_square.cpp` — square target generator.

### Experiment support

- `swing_error_eval.cpp` — one-shot quantitative evaluator; no control output.
- `docs/EXPERIMENT_WORKFLOW.md` — fixed vs dynamic delay experiment order.

## Cleanup principle

The normal runtime graph contains only interfaces needed by the active estimator and controller. Optional comparison estimators and experiment tools remain available but are not launched by default.
