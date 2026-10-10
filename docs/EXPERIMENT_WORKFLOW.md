# Experiment workflow

This file defines the repeatable startup order for fixed-delay vs dynamic-delay Figure-8 comparisons.

## A. Fixed-delay baseline

1. Start `uav_slung_sim.launch.py`.
2. Start `swing_angle.launch.py`.
3. Set:

   ```bash
   ros2 param set /swing_angle_eso_fusion use_dynamic_delay false
   ```

4. Start `slung_antiswing.launch.py` and wait for stable CONTROL.
5. Start evaluator:

   ```bash
   ros2 run slung_ctrl swing_error_eval --ros-args \
     -p experiment_name:=fixed_tau30 \
     -p duration_sec:=35.0 \
     -p output_csv:=/tmp/fixed_tau30.csv
   ```

6. Start `slung_antiswing_eight.launch.py`.
7. Save evaluator terminal output and `/tmp/fixed_tau30.csv`.

## B. Dynamic-delay case

Restart the entire simulation so the initial condition is independent of A.
Repeat the same sequence, changing only:

```bash
ros2 param set /swing_angle_eso_fusion use_dynamic_delay true
```

and evaluator labels:

```bash
ros2 run slung_ctrl swing_error_eval --ros-args \
  -p experiment_name:=dynamic_tau \
  -p duration_sec:=35.0 \
  -p output_csv:=/tmp/dynamic_tau.csv
```

## Comparison metrics

Keep ADRC, trajectory, estimator bandwidth, filters, limits, and takeoff settings unchanged between A and B.

Compare:

- Estimation East/North RMSE and horizontal error RMS.
- True horizontal swing RMS, P95, and maximum.
- Horizontal position tracking RMS, P95, and maximum.
- State age statistics.
- Time-series curves from the two CSV files.

## Current limitation

This comparison is a Stage-1 check. The current fusion observer still reuses the most recent low-rate visual sample in multiple high-rate IMU updates. Final paper-quality comparisons should be repeated after the multi-rate predictor/corrector structure is implemented.
