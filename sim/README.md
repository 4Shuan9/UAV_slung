# Simulation assets

This directory contains the Gazebo resources that were previously stored under
`PX4-Autopilot_PATCH/Tools/simulation/gz/`.

They are now project-local:

```text
sim/
├── models/
└── worlds/
```

`slung_gazebo.launch.py` adds `sim/models` and the configured PX4 Gazebo model
directory to `GZ_SIM_RESOURCE_PATH`, so custom project assets no longer need to
be copied into the PX4 source tree just to launch the simulation.

The world `UAV_slung_world.sdf` still references the vehicle model
`x500_mono_cam_down`. The uploaded simulation archive did not contain that
model directory itself; it is therefore expected to be provided by the existing
PX4/Gazebo environment (as in the original working setup) or another Gazebo
resource path.
