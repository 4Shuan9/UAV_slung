// ==============================================================================
// @File     : slung_control_adrc.cpp
// @Author   : 4Shuan9
// @Date     : 2026-09-16
// @Brief    : 核心飞行控制与吊挂防摆 ADRC 节点
//             East/North + synchronized LESO state + runtime ablation parameters
//
// @Note     : 1) 保持现有 ADRC 数学结构和状态机不变；
//             2) 新增 enable_rate_term / enable_disturbance_comp 两个运行时参数；
//             3) disturbance_gain 与限幅/滤波参数支持运行时调节；
//             4) ADRC consumes theta/rate/disturbance from one synchronized message;
//             5) disturbance 链路：raw F -> 预限幅 -> 一阶低通 -> gain -> 最终限幅；
//             6) 所有限幅/滤波均可运行时开关与调参；
//
// Coordinate conventions:
//   Gazebo World: X = East, Y = North
//   PX4 local NED: X = North, Y = East
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "slung_ctrl/msg/swing_adrc_state.hpp"
#include "slung_ctrl/ekf3.hpp"

#include <cmath>
#include <mutex>
#include <algorithm>

using namespace std::chrono_literals;

enum class FlightState { INIT, TAKEOFF, CONTROL };

class SlungAntiswing : public rclcpp::Node {
public:
    SlungAntiswing() : Node("slung_control_adrc") {
        // ----------------------------------------------------------------------
        // Flight / position-loop parameters
        // ----------------------------------------------------------------------
        this->declare_parameter("takeoff_height", 10.0);
        this->declare_parameter("takeoff_speed", 0.5);
        this->declare_parameter("kp_pos", 1.2);
        this->declare_parameter("kd_pos", 0.8);

        // ----------------------------------------------------------------------
        // ADRC parameters
        // ----------------------------------------------------------------------
        this->declare_parameter("omega_c", 3.0);      // controller bandwidth
        this->declare_parameter("zeta", 0.10);        // damping ratio
        this->declare_parameter("l", 1.0);            // cable length [m]
        this->declare_parameter("g", 9.81);           // gravity [m/s^2]
        this->declare_parameter("max_comp_acc", 3.0); // total horizontal acceleration limit [m/s^2]

        // Phase-1 EKF interface. Legacy launch keeps its original source/law.
        this->declare_parameter("state_topic", "/uav/swing/eso_fusion/state");
        this->declare_parameter("nonlinear_antiswing", false);
        this->declare_parameter("max_state_age", 0.10);
        this->declare_parameter("max_swing_angle_deg", 35.0);
        const double max_state_age = this->get_parameter("max_state_age").as_double();
        const double max_angle = this->get_parameter("max_swing_angle_deg").as_double();
        if (!std::isfinite(max_state_age) || max_state_age <= 0 ||
            !std::isfinite(max_angle) || max_angle <= 0 || max_angle >= 89.0) {
            throw std::invalid_argument("Invalid state freshness/angle limits");
        }

        // ----------------------------------------------------------------------
        // Ablation switches
        // Defaults are true so that the controller behaves exactly like the
        // previous ADRC version unless explicitly disabled at runtime.
        // ----------------------------------------------------------------------
        this->declare_parameter("enable_rate_term", true);
        this->declare_parameter("enable_disturbance_comp", true);
        this->declare_parameter("disturbance_gain", 0.50);

        // Disturbance-only low-pass filter.
        // Continuous-time form: tau_f * y_dot + y = x
        // Discrete update (variable dt): alpha = dt / (tau_f + dt)
        // Default tau_f = 0.05 s -> conservative smoothing without assuming
        // a fixed LESO publication rate.
        this->declare_parameter("enable_disturbance_filter", true);
        this->declare_parameter("disturbance_filter_tau", 0.05); // [s]

        // Disturbance protection chain:
        //   raw f_hat [rad/s^2]
        //       -> wide pre-limit (outlier protection)
        //       -> LPF
        //       -> disturbance_gain
        //       -> final used-F limit [m/s^2]
        //
        // Based on the current measured figure-8 baseline:
        // raw-F MaxAbs ~= 0.186 East / 0.242 North rad/s^2.
        // 0.30 rad/s^2 therefore leaves normal dynamic headroom while rejecting
        // clearly abnormal spikes before they contaminate the LPF state.
        this->declare_parameter("enable_disturbance_prelimit", true);
        this->declare_parameter("disturbance_raw_limit", 0.30); // [rad/s^2]

        // Final safety limit on the disturbance contribution that is ACTUALLY
        // injected into the controller after LPF and disturbance_gain.
        // This is a final safety guard on the disturbance term actually injected.
        this->declare_parameter("enable_disturbance_used_limit", true);
        this->declare_parameter("disturbance_used_limit", 0.15); // [m/s^2]

        // ----------------------------------------------------------------------
        // PX4 publishers
        // ----------------------------------------------------------------------
        pub_offboard_mode = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        pub_trajectory = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        pub_cmd = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // ----------------------------------------------------------------------
        // Experiment-state publisher
        // ----------------------------------------------------------------------
        // Explicit controller-state flag for experiments/evaluation.
        // false during INIT/TAKEOFF, true only after the configured takeoff
        // height has been reached and the anti-swing CONTROL state is active.
        pub_control_active = this->create_publisher<std_msgs::msg::Bool>(
            "/uav/control/active", 10);

        // ----------------------------------------------------------------------
        // Synchronized LESO state subscription
        // theta / rate / disturbance now arrive as ONE coherent snapshot.
        // ----------------------------------------------------------------------
        sub_adrc_state = this->create_subscription<slung_ctrl::msg::SwingADRCState>(
            this->get_parameter("state_topic").as_string(), 10,
            std::bind(&SlungAntiswing::adrc_state_cb, this, std::placeholders::_1));

        // ----------------------------------------------------------------------
        // PX4 odometry / external target subscriptions
        // ----------------------------------------------------------------------
        sub_odom = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&SlungAntiswing::odom_cb, this, std::placeholders::_1));

        sub_target_pos = this->create_subscription<geometry_msgs::msg::Point>(
            "/uav/reference/position", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                if (state_ == FlightState::CONTROL) {
                    // /uav/reference/position follows PX4 local horizontal semantics:
                    // x -> North, y -> East
                    target_north = msg->x;
                    target_east = msg->y;
                    target_z = msg->z;
                }
            });

        sub_target_yaw = this->create_subscription<std_msgs::msg::Float32>(
            "/uav/reference/yaw", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (state_ == FlightState::CONTROL) {
                    target_yaw = msg->data;
                }
            });

        timer = this->create_wall_timer(20ms, std::bind(&SlungAntiswing::control_loop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "ADRC Node Init (Synchronized LESO State + East/North). Waiting for Odom...");
    }

private:
    FlightState state_ = FlightState::INIT;
    int offboard_counter = 0;
    std::mutex data_mutex;

    // Synchronized ESO Fusion state. Units intentionally match the previous
    // three-topic interface during this stage: theta [deg], rate [deg/s],
    // disturbance [rad/s^2].
    double theta_east_deg = 0.0;
    double theta_north_deg = 0.0;
    double rate_east_deg_s = 0.0;
    double rate_north_deg_s = 0.0;
    double disturbance_east = 0.0;
    double disturbance_north = 0.0;
    double disturbance_east_prelimited = 0.0;
    double disturbance_north_prelimited = 0.0;

    // Disturbance LPF states [rad/s^2].  These are updated at the synchronized
    // LESO-state callback rate (about 200 Hz in the current setup), not at the
    // 50 Hz flight-control timer.
    double disturbance_east_filtered = 0.0;
    double disturbance_north_filtered = 0.0;
    bool disturbance_filter_initialized = false;
    double last_adrc_state_stamp_sec = 0.0;

    // PX4 local NED horizontal states:
    // position[0]/velocity[0] -> North
    // position[1]/velocity[1] -> East
    float current_north = 0.0f;
    float current_east = 0.0f;
    float current_z = 0.0f;
    float current_v_north = 0.0f;
    float current_v_east = 0.0f;

    float target_north = 0.0f;
    float target_east = 0.0f;
    float target_z = 0.0f;
    float target_yaw = 0.0f;
    float current_sp_z = 0.0f;

    bool odom_received = false;
    bool is_mid_air = false;
    bool adrc_state_received = false;
    double last_valid_state_time = -1.0;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_trajectory;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_control_active;

    rclcpp::Subscription<slung_ctrl::msg::SwingADRCState>::SharedPtr sub_adrc_state;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_pos;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_yaw;
    rclcpp::TimerBase::SharedPtr timer;

    void adrc_state_cb(const slung_ctrl::msg::SwingADRCState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex);

        const double stamp_time = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type()).seconds();
        const double age = this->get_clock()->now().seconds() - stamp_time;
        for (double value : {msg->theta_east_deg, msg->theta_north_deg,
                            msg->rate_east_deg_s, msg->rate_north_deg_s,
                            msg->disturbance_east_rad_s2, msg->disturbance_north_rad_s2}) {
            if (!std::isfinite(value)) return;
        }
        const double max_angle = this->get_parameter("max_swing_angle_deg").as_double();
        if (!std::isfinite(age) || age < 0 || age > this->get_parameter("max_state_age").as_double() ||
            std::abs(msg->theta_east_deg) > max_angle || std::abs(msg->theta_north_deg) > max_angle) return;
        last_valid_state_time = stamp_time;

        // Atomic snapshot: theta/rate/f belong to the same estimator update.
        theta_east_deg = msg->theta_east_deg;
        theta_north_deg = msg->theta_north_deg;
        rate_east_deg_s = msg->rate_east_deg_s;
        rate_north_deg_s = msg->rate_north_deg_s;
        disturbance_east = msg->disturbance_east_rad_s2;
        disturbance_north = msg->disturbance_north_rad_s2;

        // ------------------------------------------------------------------
        // Wide raw-F pre-limit [rad/s^2].
        // This is NOT intended to shape normal disturbance dynamics.  It only
        // prevents a rare observer/vision spike from charging the LPF state and
        // creating a long artificial tail.
        // ------------------------------------------------------------------
        const bool enable_prelimit =
            this->get_parameter("enable_disturbance_prelimit").as_bool();
        const double raw_limit = std::max(
            0.0, this->get_parameter("disturbance_raw_limit").as_double());

        if (enable_prelimit && raw_limit > 0.0) {
            disturbance_east_prelimited =
                std::clamp(disturbance_east, -raw_limit, raw_limit);
            disturbance_north_prelimited =
                std::clamp(disturbance_north, -raw_limit, raw_limit);
        } else {
            disturbance_east_prelimited = disturbance_east;
            disturbance_north_prelimited = disturbance_north;
        }

        // ------------------------------------------------------------------
        // Stage 2: disturbance-only first-order low-pass filter.
        //
        // Use the synchronized LESO message timestamp to calculate the real dt,
        // so the filter does not assume exactly 200 Hz.  On the first sample,
        // or after an abnormal timestamp gap, initialize from the raw value to
        // avoid a long artificial startup transient.
        // ------------------------------------------------------------------
        const double stamp_sec =
            static_cast<double>(msg->header.stamp.sec)
            + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

        const double tau_f = std::max(
            0.0, this->get_parameter("disturbance_filter_tau").as_double());

        double dt = 0.0;
        if (disturbance_filter_initialized) {
            dt = stamp_sec - last_adrc_state_stamp_sec;
        }

        const bool valid_dt = (dt > 0.0 && dt < 0.1);

        if (!disturbance_filter_initialized || !valid_dt || tau_f <= 0.0) {
            disturbance_east_filtered = disturbance_east_prelimited;
            disturbance_north_filtered = disturbance_north_prelimited;
            disturbance_filter_initialized = true;
        } else {
            const double alpha = dt / (tau_f + dt);
            disturbance_east_filtered +=
                alpha * (disturbance_east_prelimited - disturbance_east_filtered);
            disturbance_north_filtered +=
                alpha * (disturbance_north_prelimited - disturbance_north_filtered);
        }

        last_adrc_state_stamp_sec = stamp_sec;
        adrc_state_received = true;
    }

    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        current_north = msg->position[0];
        current_east = msg->position[1];
        current_z = msg->position[2];

        current_v_north = msg->velocity[0];
        current_v_east = msg->velocity[1];

        if (!odom_received) {
            target_north = current_north;
            target_east = current_east;
            current_sp_z = current_z;

            float w = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];
            target_yaw = std::atan2(
                2.0f * (w * z + x * y),
                1.0f - 2.0f * (y * y + z * z));

            if (current_z < -0.5f) {
                is_mid_air = true;
                RCLCPP_INFO(
                    this->get_logger(),
                    ">>> Mid-air Restart Detected! Resuming Hover at Z: %.2fm",
                    current_z);
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Ground Start Detected. Initial Yaw: %.4f rad.",
                    target_yaw);
            }

            odom_received = true;
        }
    }

    void control_loop() {
        // Publish the real state-machine status every control tick.  This lets
        // evaluation nodes ignore ground/takeoff data automatically.
        std_msgs::msg::Bool active_msg;
        active_msg.data = (state_ == FlightState::CONTROL);
        pub_control_active->publish(active_msg);

        if (!odom_received) {
            return;
        }

        publish_offboard_control_mode();

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        sp.yaw = target_yaw;

        if (state_ == FlightState::INIT) {
            sp.position = {target_north, target_east, current_sp_z};
            sp.velocity = {NAN, NAN, NAN};
            sp.acceleration = {NAN, NAN, NAN};
            pub_trajectory->publish(sp);

            if (++offboard_counter == 15) {
                publish_vehicle_command(
                    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(
                    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

                target_z = is_mid_air
                    ? current_sp_z
                    : -(this->get_parameter("takeoff_height").as_double());
            }

            if (offboard_counter >= 20) {
                if (is_mid_air) {
                    state_ = FlightState::CONTROL;
                    RCLCPP_INFO(this->get_logger(), "Anti-Swing Resumed Seamlessly.");
                } else {
                    state_ = FlightState::TAKEOFF;
                }
            }
            return;
        }

        if (state_ == FlightState::TAKEOFF) {
            const double speed = this->get_parameter("takeoff_speed").as_double();

            if (current_sp_z > target_z) {
                current_sp_z -= (speed * 0.02);
            } else {
                current_sp_z = target_z;
                if (std::abs(current_z - target_z) < 0.3) {
                    state_ = FlightState::CONTROL;
                    RCLCPP_INFO(this->get_logger(), "Takeoff Done. Full ADRC Active.");
                }
            }

            sp.position = {target_north, target_east, current_sp_z};
            sp.velocity = {0.0f, 0.0f, (float)-speed};
            sp.acceleration = {NAN, NAN, NAN};
            pub_trajectory->publish(sp);
            return;
        }

        if (state_ == FlightState::CONTROL) {
            const double kp_pos = this->get_parameter("kp_pos").as_double();
            const double kd_pos = this->get_parameter("kd_pos").as_double();

            const double omega_c = this->get_parameter("omega_c").as_double();
            const double zeta = this->get_parameter("zeta").as_double();
            const double l = this->get_parameter("l").as_double();
            const double g = this->get_parameter("g").as_double();
            const double a_max = this->get_parameter("max_comp_acc").as_double();

            const bool enable_rate_term =
                this->get_parameter("enable_rate_term").as_bool();
            const bool enable_disturbance_comp =
                this->get_parameter("enable_disturbance_comp").as_bool();
            const double disturbance_gain =
                this->get_parameter("disturbance_gain").as_double();
            const bool enable_disturbance_filter =
                this->get_parameter("enable_disturbance_filter").as_bool();
            const bool enable_disturbance_used_limit =
                this->get_parameter("enable_disturbance_used_limit").as_bool();
            const double disturbance_used_limit = std::max(
                0.0, this->get_parameter("disturbance_used_limit").as_double());

            // Dynamic ADRC gains
            const double kp_swing = std::pow(omega_c, 2);
            const double kd_swing = 2.0 * zeta * omega_c;

            double theta_east_rad;
            double theta_north_rad;
            double rate_east_rad_s;
            double rate_north_rad_s;
            double disturbance_east_local;
            double disturbance_north_local;
            double disturbance_east_filtered_local;
            double disturbance_north_filtered_local;
            bool has_synced_state_local;

            {
                std::lock_guard<std::mutex> lock(data_mutex);
                const double age = this->get_clock()->now().seconds() - last_valid_state_time;
                has_synced_state_local = adrc_state_received && age >= 0 &&
                    age <= this->get_parameter("max_state_age").as_double();

                theta_east_rad = theta_east_deg * M_PI / 180.0;
                theta_north_rad = theta_north_deg * M_PI / 180.0;

                rate_east_rad_s = rate_east_deg_s * M_PI / 180.0;
                rate_north_rad_s = rate_north_deg_s * M_PI / 180.0;

                disturbance_east_local = disturbance_east;
                disturbance_north_local = disturbance_north;
                disturbance_east_filtered_local = disturbance_east_filtered;
                disturbance_north_filtered_local = disturbance_north_filtered;
            }

            // Before the first synchronized state arrives, keep anti-swing terms
            // at zero rather than using a mixed or partially initialized state.
            if (!has_synced_state_local) {
                theta_east_rad = 0.0;
                theta_north_rad = 0.0;
                rate_east_rad_s = 0.0;
                rate_north_rad_s = 0.0;
                disturbance_east_local = 0.0;
                disturbance_north_local = 0.0;
                disturbance_east_filtered_local = 0.0;
                disturbance_north_filtered_local = 0.0;
            }

            // ------------------------------------------------------------------
            // ADRC anti-swing terms [m/s^2]
            //
            // P_swing = l * (kp - g/l) * theta_hat
            // D_swing = l * kd * theta_dot_hat
            // F_comp  = l * f_hat
            // ------------------------------------------------------------------
            const double p_swing_north =
                l * (kp_swing - g / l) * theta_north_rad;
            const double p_swing_east =
                l * (kp_swing - g / l) * theta_east_rad;

            const double d_swing_north_raw =
                l * kd_swing * rate_north_rad_s;
            const double d_swing_east_raw =
                l * kd_swing * rate_east_rad_s;

            const double f_comp_north_raw =
                l * disturbance_north_local;
            const double f_comp_east_raw =
                l * disturbance_east_local;

            const double f_comp_north_filtered =
                l * disturbance_north_filtered_local;
            const double f_comp_east_filtered =
                l * disturbance_east_filtered_local;

            const double d_swing_north =
                enable_rate_term ? d_swing_north_raw : 0.0;
            const double d_swing_east =
                enable_rate_term ? d_swing_east_raw : 0.0;

            // Select raw or filtered disturbance BEFORE disturbance_gain.
            // This keeps the A/B test clean:
            //   filter OFF -> identical disturbance path to the previous version
            //   filter ON  -> only the F path gains one first-order LPF
            const double f_comp_north_selected =
                enable_disturbance_filter
                    ? f_comp_north_filtered
                    : f_comp_north_raw;
            const double f_comp_east_selected =
                enable_disturbance_filter
                    ? f_comp_east_filtered
                    : f_comp_east_raw;

            // disturbance_gain = 1.0 -> full selected disturbance compensation
            // disturbance_gain = 0.0 -> disturbance compensation disabled
            // Intermediate values remain available for robust A/B testing.
            const double f_comp_north_unlimited =
                enable_disturbance_comp
                    ? disturbance_gain * f_comp_north_selected
                    : 0.0;
            const double f_comp_east_unlimited =
                enable_disturbance_comp
                    ? disturbance_gain * f_comp_east_selected
                    : 0.0;

            // Final safety limit on the ACTUAL F contribution [m/s^2].
            // At the current baseline, the configured limit
            // should normally stay inactive.  It is intentionally a guardrail,
            // not a normal-operation shaping element.
            double f_comp_north = f_comp_north_unlimited;
            double f_comp_east = f_comp_east_unlimited;

            if (enable_disturbance_used_limit && disturbance_used_limit > 0.0) {
                f_comp_north = std::clamp(
                    f_comp_north_unlimited,
                    -disturbance_used_limit,
                    disturbance_used_limit);
                f_comp_east = std::clamp(
                    f_comp_east_unlimited,
                    -disturbance_used_limit,
                    disturbance_used_limit);
            }

            const bool nonlinear = this->get_parameter("nonlinear_antiswing").as_bool();
            const double max_angle_rad = this->get_parameter("max_swing_angle_deg").as_double() * M_PI / 180.0;
            const bool valid_gains = std::isfinite(l) && l > 0 && std::isfinite(g) && g > 0 &&
                std::isfinite(omega_c) && omega_c > 0 && std::isfinite(zeta) && zeta > 0 &&
                std::isfinite(a_max) && a_max > 0 && std::isfinite(max_angle_rad) &&
                max_angle_rad > 0 && max_angle_rad < M_PI / 2;
            if (!valid_gains) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Invalid controller gains");
                return;
            }
            const auto antiswing = [&](double theta, double rate, double f_acc, double linear) {
                if (!has_synced_state_local || std::abs(theta) > max_angle_rad) return 0.0;
                if (!nonlinear) return linear;
                return slung_ctrl::antiswing_acceleration(theta,
                    enable_rate_term ? rate : 0.0, f_acc / l, l, g, kp_swing, kd_swing);
            };
            const float a_anti_north = static_cast<float>(antiswing(theta_north_rad,
                rate_north_rad_s, f_comp_north, p_swing_north + d_swing_north + f_comp_north));
            const float a_anti_east = static_cast<float>(antiswing(theta_east_rad,
                rate_east_rad_s, f_comp_east, p_swing_east + d_swing_east + f_comp_east));
            if (!has_synced_state_local) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Estimator state unavailable/stale; position PD only");
            }

            // Position outer loop + anti-swing compensation
            const float accel_north =
                kp_pos * (target_north - current_north)
                - kd_pos * current_v_north
                + a_anti_north;

            const float accel_east =
                kp_pos * (target_east - current_east)
                - kd_pos * current_v_east
                + a_anti_east;

            sp.position = {NAN, NAN, target_z};
            sp.velocity = {NAN, NAN, NAN};
            sp.acceleration = {
                std::clamp(accel_north, (float)-a_max, (float)a_max),
                std::clamp(accel_east, (float)-a_max, (float)a_max),
                NAN
            };

            pub_trajectory->publish(sp);
        }
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        pub_offboard_mode->publish(msg);
    }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0.0, float p2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = p1;
        msg.param2 = p2;
        msg.command = cmd;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub_cmd->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlungAntiswing>());
    rclcpp::shutdown();
    return 0;
}
