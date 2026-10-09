// ==============================================================================
// @File  : swing_error_eval.cpp
// @Brief : One-shot quantitative evaluator for UAV slung-load ADRC experiments.
//
// What it measures during the requested recording window:
//   A) Estimation quality: ESO angle - Gazebo truth [deg]
//   B) True anti-swing performance: truth swing angle itself [deg]
//   C) Position tracking: target - PX4 odometry [m]
//   D) Disturbance/control quality:
//        LESO f_hat [rad/s^2]
//        raw / prelimited / filtered / used_unlimited / used F terms [m/s^2]
//   E) Limiter activity and synchronized-state age.
//
// Ground/takeoff is deliberately excluded.
// Sampling starts only after /uav/adrc_debug/control_active == true,
// then waits settle_sec.
//
// Coordinate conventions:
//   Gazebo world / public swing/debug Vector3:
//       vector.x = East, vector.y = North
//   PX4 local NED:
//       position[0] = North, position[1] = East
//   /uav/target_position:
//       x = North, y = East
//
// Error conventions:
//   estimation error = estimate - truth
//   position error   = target - actual
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "slung_ctrl/msg/swing_adrc_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

class SwingErrorEval : public rclcpp::Node {
public:
    SwingErrorEval() : Node("swing_error_eval") {
        declare_parameter("experiment_name", std::string("experiment"));
        declare_parameter("duration_sec", 30.0);
        declare_parameter("settle_sec", 5.0);
        declare_parameter("max_state_age_ms", 20.0);
        declare_parameter("output_csv", std::string("/tmp/swing_control_eval.csv"));
        declare_parameter("auto_shutdown", true);

        // These two values are used only to REPORT limiter activity.
        // Keep them equal to the controller parameters for the current test.
        declare_parameter("disturbance_raw_limit", 0.30);   // [rad/s^2] when l=1;
                                                             // debug prelimit term is [m/s^2]
        declare_parameter("disturbance_used_limit", 0.08);  // [m/s^2]
        declare_parameter("limit_epsilon", 1.0e-6);

        experiment_name_ = get_parameter("experiment_name").as_string();
        duration_sec_ = get_parameter("duration_sec").as_double();
        settle_sec_ = get_parameter("settle_sec").as_double();
        max_state_age_sec_ = get_parameter("max_state_age_ms").as_double() / 1000.0;
        output_csv_ = get_parameter("output_csv").as_string();
        auto_shutdown_ = get_parameter("auto_shutdown").as_bool();
        disturbance_raw_limit_ = std::abs(get_parameter("disturbance_raw_limit").as_double());
        disturbance_used_limit_ = std::abs(get_parameter("disturbance_used_limit").as_double());
        limit_epsilon_ = std::abs(get_parameter("limit_epsilon").as_double());

        sub_active_ = create_subscription<std_msgs::msg::Bool>(
            "/uav/adrc_debug/control_active", 10,
            std::bind(&SwingErrorEval::active_cb, this, std::placeholders::_1));

        sub_state_ = create_subscription<slung_ctrl::msg::SwingADRCState>(
            "/uav/eso_fusion/adrc_state", 10,
            std::bind(&SwingErrorEval::state_cb, this, std::placeholders::_1));

        sub_truth_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/truth/swing_angle/world_frame/cartesian", 10,
            std::bind(&SwingErrorEval::truth_cb, this, std::placeholders::_1));

        sub_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&SwingErrorEval::odom_cb, this, std::placeholders::_1));

        sub_target_ = create_subscription<geometry_msgs::msg::Point>(
            "/uav/target_position", 10,
            std::bind(&SwingErrorEval::target_cb, this, std::placeholders::_1));

        sub_f_raw_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/adrc_debug/f_comp/cartesian", 10,
            std::bind(&SwingErrorEval::f_raw_cb, this, std::placeholders::_1));

        sub_f_prelimited_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/adrc_debug/f_comp_prelimited/cartesian", 10,
            std::bind(&SwingErrorEval::f_prelimited_cb, this, std::placeholders::_1));

        sub_f_filtered_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/adrc_debug/f_comp_filtered/cartesian", 10,
            std::bind(&SwingErrorEval::f_filtered_cb, this, std::placeholders::_1));

        sub_f_used_unlimited_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/adrc_debug/f_comp_used_unlimited/cartesian", 10,
            std::bind(&SwingErrorEval::f_used_unlimited_cb, this, std::placeholders::_1));

        sub_f_used_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/adrc_debug/f_comp_used/cartesian", 10,
            std::bind(&SwingErrorEval::f_used_cb, this, std::placeholders::_1));

        csv_.open(output_csv_, std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            RCLCPP_WARN(
                get_logger(),
                "Could not open CSV: %s (terminal summary will still work)",
                output_csv_.c_str());
        } else {
            csv_
                << "elapsed_sec,"
                << "truth_east_deg,est_east_deg,est_error_east_deg,"
                << "truth_north_deg,est_north_deg,est_error_north_deg,"
                << "leso_fhat_east_rad_s2,leso_fhat_north_rad_s2,"
                << "target_east_m,actual_east_m,pos_error_east_m,"
                << "target_north_m,actual_north_m,pos_error_north_m,"
                << "f_raw_east_m_s2,f_raw_north_m_s2,"
                << "f_prelimited_east_m_s2,f_prelimited_north_m_s2,"
                << "f_filtered_east_m_s2,f_filtered_north_m_s2,"
                << "f_used_unlimited_east_m_s2,f_used_unlimited_north_m_s2,"
                << "f_used_east_m_s2,f_used_north_m_s2,"
                << "state_age_ms\n";
        }

        RCLCPP_INFO(
            get_logger(),
            "Evaluator [%s] armed. Waiting for ADRC CONTROL. Then wait %.1f s and record %.1f s. "
            "max_state_age=%.1f ms | raw_limit=%.3f | used_limit=%.3f",
            experiment_name_.c_str(),
            settle_sec_,
            duration_sec_,
            max_state_age_sec_ * 1000.0,
            disturbance_raw_limit_,
            disturbance_used_limit_);
    }

private:
    struct ErrorStats {
        double bias = std::numeric_limits<double>::quiet_NaN();
        double mae = std::numeric_limits<double>::quiet_NaN();
        double rmse = std::numeric_limits<double>::quiet_NaN();
        double stddev = std::numeric_limits<double>::quiet_NaN();
        double max_abs = std::numeric_limits<double>::quiet_NaN();
        double p95_abs = std::numeric_limits<double>::quiet_NaN();
    };

    struct SignalStats {
        double mean = std::numeric_limits<double>::quiet_NaN();
        double rms = std::numeric_limits<double>::quiet_NaN();
        double stddev = std::numeric_limits<double>::quiet_NaN();
        double max_abs = std::numeric_limits<double>::quiet_NaN();
        double p95_abs = std::numeric_limits<double>::quiet_NaN();
    };

    bool control_active_ = false;
    bool have_state_ = false;
    bool have_target_ = false;
    bool have_odom_ = false;
    bool measuring_ = false;
    bool finished_ = false;

    rclcpp::Time control_active_since_{0, 0, RCL_ROS_TIME};
    rclcpp::Time state_rx_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time measure_start_{0, 0, RCL_ROS_TIME};

    slung_ctrl::msg::SwingADRCState latest_state_;

    double target_north_ = 0.0;
    double target_east_ = 0.0;
    double actual_north_ = 0.0;
    double actual_east_ = 0.0;

    double latest_f_raw_e_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_raw_n_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_pre_e_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_pre_n_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_filt_e_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_filt_n_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_unlim_e_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_unlim_n_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_used_e_ = std::numeric_limits<double>::quiet_NaN();
    double latest_f_used_n_ = std::numeric_limits<double>::quiet_NaN();

    std::string experiment_name_;
    double duration_sec_ = 30.0;
    double settle_sec_ = 5.0;
    double max_state_age_sec_ = 0.020;
    std::string output_csv_;
    bool auto_shutdown_ = true;
    double disturbance_raw_limit_ = 0.30;
    double disturbance_used_limit_ = 0.08;
    double limit_epsilon_ = 1.0e-6;

    // A) Estimation
    std::vector<double> err_east_;
    std::vector<double> err_north_;

    // B) True anti-swing performance
    std::vector<double> truth_east_;
    std::vector<double> truth_north_;
    std::vector<double> truth_horizontal_mag_;

    // LESO f_hat as observer output
    std::vector<double> fhat_east_;
    std::vector<double> fhat_north_;

    // C) Position tracking
    std::vector<double> pos_err_east_;
    std::vector<double> pos_err_north_;
    std::vector<double> pos_err_horizontal_mag_;

    // D) F-chain, each recorded at its own debug-topic rate.
    std::vector<double> f_raw_east_;
    std::vector<double> f_raw_north_;
    std::vector<double> f_pre_east_;
    std::vector<double> f_pre_north_;
    std::vector<double> f_filt_east_;
    std::vector<double> f_filt_north_;
    std::vector<double> f_unlim_east_;
    std::vector<double> f_unlim_north_;
    std::vector<double> f_used_east_;
    std::vector<double> f_used_north_;

    uint64_t prelimit_samples_ = 0;
    uint64_t prelimit_hit_east_ = 0;
    uint64_t prelimit_hit_north_ = 0;
    uint64_t prelimit_hit_any_ = 0;

    uint64_t used_limit_samples_ = 0;
    uint64_t used_limit_hit_east_ = 0;
    uint64_t used_limit_hit_north_ = 0;
    uint64_t used_limit_hit_any_ = 0;

    std::vector<double> state_age_ms_;
    uint64_t skipped_stale_state_ = 0;

    std::ofstream csv_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_active_;
    rclcpp::Subscription<slung_ctrl::msg::SwingADRCState>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_truth_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_f_raw_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_f_prelimited_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_f_filtered_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_f_used_unlimited_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_f_used_;

    bool in_recording_window() const {
        return measuring_ && !finished_;
    }

    void active_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        const auto now = this->now();

        if (msg->data && !control_active_) {
            control_active_ = true;
            control_active_since_ = now;
            measuring_ = false;
            RCLCPP_INFO(
                get_logger(),
                "ADRC CONTROL active. Waiting %.1f s settling time before [%s] starts...",
                settle_sec_,
                experiment_name_.c_str());
        } else if (!msg->data && control_active_) {
            control_active_ = false;
            measuring_ = false;
            RCLCPP_WARN(
                get_logger(),
                "ADRC left CONTROL state. Measurement paused/reset.");
        }
    }

    void state_cb(const slung_ctrl::msg::SwingADRCState::SharedPtr msg) {
        latest_state_ = *msg;
        state_rx_time_ = this->now();
        have_state_ = true;
    }

    void target_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
        // /uav/target_position: x -> North, y -> East
        target_north_ = msg->x;
        target_east_ = msg->y;
        have_target_ = true;
    }

    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        actual_north_ = msg->position[0];
        actual_east_ = msg->position[1];
        have_odom_ = true;

        if (!in_recording_window() || !have_target_) {
            return;
        }

        const double e_n = target_north_ - actual_north_;
        const double e_e = target_east_ - actual_east_;

        pos_err_north_.push_back(e_n);
        pos_err_east_.push_back(e_e);
        pos_err_horizontal_mag_.push_back(std::hypot(e_e, e_n));
    }

    void f_raw_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        latest_f_raw_e_ = msg->vector.x;
        latest_f_raw_n_ = msg->vector.y;

        if (!in_recording_window()) return;
        f_raw_east_.push_back(msg->vector.x);
        f_raw_north_.push_back(msg->vector.y);
    }

    void f_prelimited_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        latest_f_pre_e_ = msg->vector.x;
        latest_f_pre_n_ = msg->vector.y;

        if (!in_recording_window()) return;

        f_pre_east_.push_back(msg->vector.x);
        f_pre_north_.push_back(msg->vector.y);

        ++prelimit_samples_;
        const bool hit_e =
            disturbance_raw_limit_ > 0.0 &&
            std::abs(msg->vector.x) >= disturbance_raw_limit_ - limit_epsilon_;
        const bool hit_n =
            disturbance_raw_limit_ > 0.0 &&
            std::abs(msg->vector.y) >= disturbance_raw_limit_ - limit_epsilon_;

        if (hit_e) ++prelimit_hit_east_;
        if (hit_n) ++prelimit_hit_north_;
        if (hit_e || hit_n) ++prelimit_hit_any_;
    }

    void f_filtered_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        latest_f_filt_e_ = msg->vector.x;
        latest_f_filt_n_ = msg->vector.y;

        if (!in_recording_window()) return;
        f_filt_east_.push_back(msg->vector.x);
        f_filt_north_.push_back(msg->vector.y);
    }

    void f_used_unlimited_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        latest_f_unlim_e_ = msg->vector.x;
        latest_f_unlim_n_ = msg->vector.y;

        if (!in_recording_window()) return;
        f_unlim_east_.push_back(msg->vector.x);
        f_unlim_north_.push_back(msg->vector.y);
    }

    void f_used_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        latest_f_used_e_ = msg->vector.x;
        latest_f_used_n_ = msg->vector.y;

        if (!in_recording_window()) return;

        f_used_east_.push_back(msg->vector.x);
        f_used_north_.push_back(msg->vector.y);

        ++used_limit_samples_;
        const bool hit_e =
            disturbance_used_limit_ > 0.0 &&
            std::abs(msg->vector.x) >= disturbance_used_limit_ - limit_epsilon_;
        const bool hit_n =
            disturbance_used_limit_ > 0.0 &&
            std::abs(msg->vector.y) >= disturbance_used_limit_ - limit_epsilon_;

        if (hit_e) ++used_limit_hit_east_;
        if (hit_n) ++used_limit_hit_north_;
        if (hit_e || hit_n) ++used_limit_hit_any_;
    }

    void truth_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        if (finished_ || !control_active_ || !have_state_) {
            return;
        }

        const auto now = this->now();

        if ((now - control_active_since_).seconds() < settle_sec_) {
            return;
        }

        const double state_age = (now - state_rx_time_).seconds();
        if (state_age < 0.0 || state_age > max_state_age_sec_) {
            ++skipped_stale_state_;
            return;
        }

        if (!measuring_) {
            measuring_ = true;
            measure_start_ = now;

            RCLCPP_INFO(
                get_logger(),
                "[%s] recording STARTED. Truth + synchronized LESO are valid. Duration %.1f s. "
                "Position tracking will be included when /uav/target_position is available.",
                experiment_name_.c_str(),
                duration_sec_);
        }

        const double elapsed = (now - measure_start_).seconds();
        if (elapsed > duration_sec_) {
            finish();
            return;
        }

        const double truth_east = msg->vector.x;
        const double truth_north = msg->vector.y;
        const double est_east = latest_state_.theta_east_deg;
        const double est_north = latest_state_.theta_north_deg;
        const double err_e = est_east - truth_east;
        const double err_n = est_north - truth_north;

        err_east_.push_back(err_e);
        err_north_.push_back(err_n);

        truth_east_.push_back(truth_east);
        truth_north_.push_back(truth_north);
        truth_horizontal_mag_.push_back(std::hypot(truth_east, truth_north));

        fhat_east_.push_back(latest_state_.disturbance_east_rad_s2);
        fhat_north_.push_back(latest_state_.disturbance_north_rad_s2);

        state_age_ms_.push_back(state_age * 1000.0);

        if (csv_.is_open()) {
            const double pos_err_e =
                (have_target_ && have_odom_) ? target_east_ - actual_east_
                                             : std::numeric_limits<double>::quiet_NaN();
            const double pos_err_n =
                (have_target_ && have_odom_) ? target_north_ - actual_north_
                                             : std::numeric_limits<double>::quiet_NaN();

            csv_ << std::fixed << std::setprecision(9)
                 << elapsed << ','
                 << truth_east << ',' << est_east << ',' << err_e << ','
                 << truth_north << ',' << est_north << ',' << err_n << ','
                 << latest_state_.disturbance_east_rad_s2 << ','
                 << latest_state_.disturbance_north_rad_s2 << ','
                 << (have_target_ ? target_east_ : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (have_odom_ ? actual_east_ : std::numeric_limits<double>::quiet_NaN()) << ','
                 << pos_err_e << ','
                 << (have_target_ ? target_north_ : std::numeric_limits<double>::quiet_NaN()) << ','
                 << (have_odom_ ? actual_north_ : std::numeric_limits<double>::quiet_NaN()) << ','
                 << pos_err_n << ','
                 << latest_f_raw_e_ << ',' << latest_f_raw_n_ << ','
                 << latest_f_pre_e_ << ',' << latest_f_pre_n_ << ','
                 << latest_f_filt_e_ << ',' << latest_f_filt_n_ << ','
                 << latest_f_unlim_e_ << ',' << latest_f_unlim_n_ << ','
                 << latest_f_used_e_ << ',' << latest_f_used_n_ << ','
                 << state_age * 1000.0 << '\n';
        }
    }

    static double percentile_abs(const std::vector<double>& values, double p) {
        if (values.empty()) return std::numeric_limits<double>::quiet_NaN();

        std::vector<double> a;
        a.reserve(values.size());

        for (double v : values) {
            a.push_back(std::abs(v));
        }

        std::sort(a.begin(), a.end());

        const double idx = p * static_cast<double>(a.size() - 1);
        const auto lo = static_cast<size_t>(std::floor(idx));
        const auto hi = static_cast<size_t>(std::ceil(idx));

        if (lo == hi) return a[lo];

        const double w = idx - static_cast<double>(lo);
        return a[lo] * (1.0 - w) + a[hi] * w;
    }

    static ErrorStats error_stats(const std::vector<double>& e) {
        ErrorStats s;
        if (e.empty()) return s;

        const double n = static_cast<double>(e.size());
        s.bias = std::accumulate(e.begin(), e.end(), 0.0) / n;

        double sum_abs = 0.0;
        double sum_sq = 0.0;
        double var_sum = 0.0;
        double max_abs = 0.0;

        for (double v : e) {
            sum_abs += std::abs(v);
            sum_sq += v * v;
            max_abs = std::max(max_abs, std::abs(v));
        }

        s.mae = sum_abs / n;
        s.rmse = std::sqrt(sum_sq / n);

        for (double v : e) {
            var_sum += (v - s.bias) * (v - s.bias);
        }

        s.stddev = std::sqrt(var_sum / n);
        s.max_abs = max_abs;
        s.p95_abs = percentile_abs(e, 0.95);
        return s;
    }

    static SignalStats signal_stats(const std::vector<double>& x) {
        SignalStats s;
        if (x.empty()) return s;

        const double n = static_cast<double>(x.size());
        s.mean = std::accumulate(x.begin(), x.end(), 0.0) / n;

        double sum_sq = 0.0;
        double var_sum = 0.0;
        double max_abs = 0.0;

        for (double v : x) {
            sum_sq += v * v;
            max_abs = std::max(max_abs, std::abs(v));
        }

        s.rms = std::sqrt(sum_sq / n);

        for (double v : x) {
            var_sum += (v - s.mean) * (v - s.mean);
        }

        s.stddev = std::sqrt(var_sum / n);
        s.max_abs = max_abs;
        s.p95_abs = percentile_abs(x, 0.95);
        return s;
    }

    static double pct(uint64_t hits, uint64_t total) {
        if (total == 0) return 0.0;
        return 100.0 * static_cast<double>(hits) / static_cast<double>(total);
    }

    void print_signal_line(
        const char* label,
        const char* unit,
        const SignalStats& s) {

        RCLCPP_INFO(
            get_logger(),
            "%-24s Mean=%+.5f RMS=%.5f Std=%.5f MaxAbs=%.5f P95Abs=%.5f %s",
            label,
            s.mean,
            s.rms,
            s.stddev,
            s.max_abs,
            s.p95_abs,
            unit);
    }

    void finish() {
        finished_ = true;
        measuring_ = false;

        if (csv_.is_open()) {
            csv_.flush();
        }

        const auto est_e = error_stats(err_east_);
        const auto est_n = error_stats(err_north_);

        const auto truth_e = signal_stats(truth_east_);
        const auto truth_n = signal_stats(truth_north_);
        const auto truth_mag = signal_stats(truth_horizontal_mag_);

        const auto fhat_e = signal_stats(fhat_east_);
        const auto fhat_n = signal_stats(fhat_north_);

        const auto pos_e = error_stats(pos_err_east_);
        const auto pos_n = error_stats(pos_err_north_);
        const auto pos_mag = signal_stats(pos_err_horizontal_mag_);

        const auto fraw_e = signal_stats(f_raw_east_);
        const auto fraw_n = signal_stats(f_raw_north_);
        const auto fpre_e = signal_stats(f_pre_east_);
        const auto fpre_n = signal_stats(f_pre_north_);
        const auto ffilt_e = signal_stats(f_filt_east_);
        const auto ffilt_n = signal_stats(f_filt_north_);
        const auto funlim_e = signal_stats(f_unlim_east_);
        const auto funlim_n = signal_stats(f_unlim_north_);
        const auto fused_e = signal_stats(f_used_east_);
        const auto fused_n = signal_stats(f_used_north_);

        const auto age = signal_stats(state_age_ms_);

        RCLCPP_INFO(
            get_logger(),
            "\n================ CONTROL EVALUATION: %s ================",
            experiment_name_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Truth/estimation samples: %zu | skipped stale state: %lu",
            err_east_.size(),
            skipped_stale_state_);

        RCLCPP_INFO(get_logger(), "---------------- A) ESTIMATION ERROR ----------------");
        RCLCPP_INFO(
            get_logger(),
            "East  [deg] Bias=%+.4f MAE=%.4f RMSE=%.4f Std=%.4f MaxAbs=%.4f P95Abs=%.4f",
            est_e.bias, est_e.mae, est_e.rmse, est_e.stddev, est_e.max_abs, est_e.p95_abs);
        RCLCPP_INFO(
            get_logger(),
            "North [deg] Bias=%+.4f MAE=%.4f RMSE=%.4f Std=%.4f MaxAbs=%.4f P95Abs=%.4f",
            est_n.bias, est_n.mae, est_n.rmse, est_n.stddev, est_n.max_abs, est_n.p95_abs);

        RCLCPP_INFO(get_logger(), "---------------- B) TRUE SWING ----------------------");
        print_signal_line("Truth East",  "[deg]", truth_e);
        print_signal_line("Truth North", "[deg]", truth_n);
        print_signal_line("Truth horizontal |theta|", "[deg]", truth_mag);

        RCLCPP_INFO(get_logger(), "---------------- C) POSITION TRACKING ---------------");
        if (pos_err_east_.empty()) {
            RCLCPP_WARN(
                get_logger(),
                "No /uav/target_position samples during the recording window. "
                "Position tracking metrics are unavailable (normal for pure hover without trajectory publisher).");
        } else {
            RCLCPP_INFO(
                get_logger(),
                "East  error [m] Bias=%+.4f MAE=%.4f RMSE=%.4f Std=%.4f MaxAbs=%.4f P95Abs=%.4f",
                pos_e.bias, pos_e.mae, pos_e.rmse, pos_e.stddev, pos_e.max_abs, pos_e.p95_abs);
            RCLCPP_INFO(
                get_logger(),
                "North error [m] Bias=%+.4f MAE=%.4f RMSE=%.4f Std=%.4f MaxAbs=%.4f P95Abs=%.4f",
                pos_n.bias, pos_n.mae, pos_n.rmse, pos_n.stddev, pos_n.max_abs, pos_n.p95_abs);
            print_signal_line("Horizontal position error", "[m]", pos_mag);
        }

        RCLCPP_INFO(get_logger(), "---------------- D) DISTURBANCE / F CHAIN -----------");
        print_signal_line("LESO f_hat East",  "[rad/s^2]", fhat_e);
        print_signal_line("LESO f_hat North", "[rad/s^2]", fhat_n);

        print_signal_line("F raw East",  "[m/s^2]", fraw_e);
        print_signal_line("F raw North", "[m/s^2]", fraw_n);
        print_signal_line("F prelimited East",  "[m/s^2]", fpre_e);
        print_signal_line("F prelimited North", "[m/s^2]", fpre_n);
        print_signal_line("F filtered East",  "[m/s^2]", ffilt_e);
        print_signal_line("F filtered North", "[m/s^2]", ffilt_n);
        print_signal_line("F used-unlimited East",  "[m/s^2]", funlim_e);
        print_signal_line("F used-unlimited North", "[m/s^2]", funlim_n);
        print_signal_line("F used East",  "[m/s^2]", fused_e);
        print_signal_line("F used North", "[m/s^2]", fused_n);

        RCLCPP_INFO(
            get_logger(),
            "Pre-limit hits: East=%lu/%lu (%.3f%%), North=%lu/%lu (%.3f%%), any-axis=%lu/%lu (%.3f%%)",
            prelimit_hit_east_, prelimit_samples_, pct(prelimit_hit_east_, prelimit_samples_),
            prelimit_hit_north_, prelimit_samples_, pct(prelimit_hit_north_, prelimit_samples_),
            prelimit_hit_any_, prelimit_samples_, pct(prelimit_hit_any_, prelimit_samples_));

        RCLCPP_INFO(
            get_logger(),
            "Used-limit hits: East=%lu/%lu (%.3f%%), North=%lu/%lu (%.3f%%), any-axis=%lu/%lu (%.3f%%)",
            used_limit_hit_east_, used_limit_samples_, pct(used_limit_hit_east_, used_limit_samples_),
            used_limit_hit_north_, used_limit_samples_, pct(used_limit_hit_north_, used_limit_samples_),
            used_limit_hit_any_, used_limit_samples_, pct(used_limit_hit_any_, used_limit_samples_));

        RCLCPP_INFO(get_logger(), "---------------- E) TIMING ---------------------------");
        RCLCPP_INFO(
            get_logger(),
            "State age [ms] Mean=%.3f RMS=%.3f MaxAbs=%.3f P95Abs=%.3f",
            age.mean, age.rms, age.max_abs, age.p95_abs);

        RCLCPP_INFO(get_logger(), "CSV: %s", output_csv_.c_str());
        RCLCPP_INFO(get_logger(), "=======================================================\n");

        if (auto_shutdown_) {
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingErrorEval>());
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
