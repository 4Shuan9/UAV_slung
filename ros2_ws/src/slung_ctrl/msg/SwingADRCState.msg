# Synchronized LESO state for ADRC control.
# One message = one observer update = one coherent control-state snapshot.
# World-frame physical directions: East / North.

std_msgs/Header header

# Keep the same units as the existing public ESO topics during this synchronization stage.
float64 theta_east_deg
float64 theta_north_deg
float64 rate_east_deg_s
float64 rate_north_deg_s

# LESO lumped disturbance estimate [rad/s^2].
float64 disturbance_east_rad_s2
float64 disturbance_north_rad_s2
