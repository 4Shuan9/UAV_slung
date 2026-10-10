# Cleanup notes — 2026-10-09

This cleanup starts from `UAV_slung_source_2026-10-09.zip` and intentionally avoids changing the core ADRC mathematics or the current fixed/dynamic delay prediction equations.

## Removed

- `scripts/gz_cam_raw.py` — explicitly deprecated camera test; functionality is already in `gz_cam_cv.py`.
- `scripts/swing_angle.py` — explicitly deprecated Python truth estimator; replaced by `swing_angle_truth.cpp`.
- `.vscode/` machine-specific settings.
- Temporary visual-delay debug topic `/uav/cv/vision_delay_ms`.
- Temporary Fusion debug topics:
  - `/uav/eso_fusion/vision_delay_debug`
  - `/uav/eso_fusion/prediction_tau_debug`
- ADRC internal P/D/F debug topic fan-out. These were useful during controller tuning but are not required for the current fixed-vs-dynamic-delay experiment.
- Unused `sensor_msgs` package dependency.

## Retained intentionally

- `vision_time_aligner.py`: required in simulation to map Gazebo image acquisition timestamps into the ROS/system-time domain used by Fusion.
- `use_dynamic_delay`, `tau`, `dynamic_delay_max`: required for fixed-vs-dynamic delay A/B tests.
- `/uav/eso_fusion/adrc_state`: the synchronized estimator snapshot consumed by ADRC.
- `/uav/adrc/control_active`: a small status interface used by the experiment evaluator.
- `swing_error_eval`: retained, but simplified to the metrics actually needed now (estimation error, true swing, position tracking, state age).
- Optional CV-comp / standard ESO / EKF estimators: retained as paper baselines, but moved out of the default estimator launch.

## Launch cleanup

`swing_angle.launch.py` now starts only the current runtime estimation chain.

Optional baseline algorithms are started separately with:

```bash
ros2 launch slung_ctrl swing_angle_baselines.launch.py
```

The controller / trajectory relationship is documented explicitly:

- `slung_antiswing.launch.py` = ADRC + PX4 Offboard + takeoff/control state machine.
- `slung_antiswing_eight.launch.py` = Figure-8 target generator only; run it alongside the ADRC controller after CONTROL is active.

## Parameter consistency

Source-code defaults for the core ADRC node were aligned with the current engineering baseline in `antiswing_params.yaml`:

- takeoff height 10 m
- omega_c 3.0
- zeta 0.10
- disturbance gain 0.50
- raw disturbance pre-limit 0.30
- filter tau 0.05 s
- used disturbance limit 0.15

Fusion default `omega_o` was aligned to 8.0, matching `eso_params.yaml`.

## Not fixed in this cleanup

The multi-rate estimator structure is deliberately left unchanged. The current Fusion node still reuses the newest ~25 Hz visual measurement inside multiple high-rate IMU callbacks. That is the next structural estimator task and should be handled as a separate controlled change.
