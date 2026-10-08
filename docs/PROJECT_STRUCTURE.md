# Project structure notes

The repository was reorganized from two independent repositories into one
project root. The merge intentionally focuses on path/layout cleanup and launch
robustness rather than changing estimator/controller algorithms.

## Main changes

- `slung_ctrl` is now under `ros2_ws/src/slung_ctrl`.
- custom Gazebo models/worlds are under `sim/` rather than a nested fake PX4 tree.
- research documents are under `docs/`.
- document images are under `media/images/research/` and Markdown references were updated.
- demo media is under `media/demo/`.
- historical data-analysis scripts and CSVs remain under `data_analysis/` so their relative paths continue to work.
- historical generated figures remain under `output/`.
- nested `.git` repositories and `.vscode` machine-specific configuration were removed.
- `slung_gazebo.launch.py` was upgraded to support configurable project/PX4 paths, staged startup and preflight checks.
- `uav_slung_sim.launch.py` was added as the preferred unified-project launch entry point.
