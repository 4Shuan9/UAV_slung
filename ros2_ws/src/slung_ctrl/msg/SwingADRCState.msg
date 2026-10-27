# Synchronized SI swing state shared by ESO/PESO, EKF3, and control.
# Unit migration 2026-10-27: angle/rate fields are rad and rad/s; old degree data requires conversion.
# One message = one observer update = one coherent control-state snapshot.
# World-frame physical directions: East / North.

std_msgs/Header header

# All public swing topics and synchronized state fields use SI units.
float64 theta_east_rad
float64 theta_north_rad
float64 rate_east_rad_s
float64 rate_north_rad_s

# Lumped disturbance [rad/s^2]; linear residual for ESO, nonlinear residual for EKF3.
float64 disturbance_east_rad_s2
float64 disturbance_north_rad_s2
