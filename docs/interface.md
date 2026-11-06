<!-- Codex-generated: moved from README.md with owner authorization. -->
# Main Interfaces

`<method>` is `eso`, `eso_delay`, `peso`, `ekf2` or `ekf3`. Swing `vector.x/y` represents **East / North**; all angle messages use **rad**, rates use **rad/s**, and disturbances use **rad/s²**. Terminal and exported figures convert angles/rates to degrees for display.

| Topic | Message type | Purpose |
|---|---|---|
| `/uav/vision/payload_pixel` | `geometry_msgs/PointStamped` | Detected floating-point center, in pixels |
| `/uav/swing/truth/angle` | `geometry_msgs/Vector3Stamped` | Ground-truth swing angles for evaluation |
| `/uav/swing/vision/angle` | `geometry_msgs/Vector3Stamped` | Visual swing-angle measurements |
| `/uav/swing/<method>/angle` | `geometry_msgs/Vector3Stamped` | Estimated swing angles |
| `/uav/swing/<method>/rate` | `geometry_msgs/Vector3Stamped` | Estimated angular rates |
| `/uav/swing/<method>/disturbance` | `geometry_msgs/Vector3Stamped` | Lumped disturbance; ESO, ESO_delay, PESO and EKF3 only |
| `/uav/swing/<method>/state` | `slung_ctrl/SwingADRCState` | Angle, rate and disturbance from one update, sharing one timestamp; ESO, ESO_delay, PESO and EKF3 only |
| `/uav/reference/angle` / `/uav/reference/rate` | `geometry_msgs/Vector3Stamped` | Angular-task references in rad / rad/s |
| `/uav/reference/position` | `geometry_msgs/Point` | Position reference in PX4 NED coordinates, in meters |
| `/uav/reference/yaw` | `std_msgs/Float32` | Yaw reference in radians, using PX4's NED convention |
| `/uav/offboard/command` | `std_msgs/String` | Command input, using the same syntax as the interactive terminal |
| `/uav/offboard/status` | `std_msgs/String` | JSON status: flight phase, task, selected observer/controller and readiness |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | PX4 estimated attitude for capture-time visual conversion |
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | Estimated acceleration: `ax` = North, `ay` = East, in m/s² |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Position and velocity feedback for the controller |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Controller setpoints sent to PX4 |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Offboard control-mode selection and heartbeat |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Vehicle commands, including arming and mode switching |

The SI interface uses `theta_east_rad`, `theta_north_rad`, `rate_east_rad_s` and `rate_north_rad_s` in `SwingADRCState`. Rebuild and restart all nodes together; archived degree-based recordings need explicit conversion. Human-facing parameters such as `vision_std_deg` retain their existing values and named units.

For PlotJuggler, raw ROS channels are radians. Right-click a plot → **Apply filter to data...** → **Scale/Offset**; set Scale to `57.29577951308232` and Offset to `0` for each angle/rate curve. Alternatively, use the Lua transform `return value * 180 / math.pi`. This changes only the display to degrees/deg/s, retaining timestamps. Save the layout to reuse the setting; the console already displays degrees.
