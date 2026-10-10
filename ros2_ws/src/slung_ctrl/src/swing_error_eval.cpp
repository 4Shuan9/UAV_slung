// ==============================================================================
// @File  : swing_error_eval.cpp
// @Brief : One-shot quantitative evaluator for UAV slung-load experiments.
//
// Measures, during one experiment window:
//   1) estimator-vs-truth swing error [deg]
//   2) true swing magnitude [deg]
//   3) horizontal trajectory tracking error [m]
//   4) synchronized estimator-state age [ms]
//
// The evaluator does not publish control commands. It only subscribes and writes
// a CSV plus a terminal summary. By default it waits for ADRC CONTROL and the
// first external target (/uav/target_position), so it naturally starts when the
// Figure-8 / square trajectory generator begins.
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
        declare_parameter("duration_sec", 35.0);
        declare_parameter("settle_sec", 3.0);
        declare_parameter("max_state_age_ms", 20.0);
        declare_parameter("require_target", true);
        declare_parameter("output_csv", std::string("/tmp/swing_control_eval.csv"));
        declare_parameter("auto_shutdown", true);

        experiment_name_ = get_parameter("experiment_name").as_string();
        duration_sec_ = get_parameter("duration_sec").as_double();
        settle_sec_ = get_parameter("settle_sec").as_double();
        max_state_age_sec_ = get_parameter("max_state_age_ms").as_double() / 1000.0;
        require_target_ = get_parameter("require_target").as_bool();
        output_csv_ = get_parameter("output_csv").as_string();
        auto_shutdown_ = get_parameter("auto_shutdown").as_bool();

        sub_active_ = create_subscription<std_msgs::msg::Bool>(
            "/uav/adrc/control_active", 10,
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
                << "truth_horizontal_deg,est_error_horizontal_deg,"
                << "target_east_m,actual_east_m,pos_error_east_m,"
                << "target_north_m,actual_north_m,pos_error_north_m,"
                << "pos_error_horizontal_m,state_age_ms\n";
        }

        RCLCPP_INFO(
            get_logger(),
            "Evaluator [%s] armed | settle=%.1f s | duration=%.1f s | require_target=%s | max_state_age=%.1f ms",
            experiment_name_.c_str(),
            settle_sec_,
            duration_sec_,
            require_target_ ? "true" : "false",
            max_state_age_sec_ * 1000.0);
    }

private:
    struct ErrorStats {
        double bias = nan();
        double mae = nan();
        double rmse = nan();
        double stddev = nan();
        double max_abs = nan();
        double p95_abs = nan();
    };

    struct SignalStats {
        double mean = nan();
        double rms = nan();
        double stddev = nan();
        double max_abs = nan();
        double p95_abs = nan();
    };

    static double nan() {
        return std::numeric_limits<double>::quiet_NaN();
    }

    static double percentile_abs(const std::vector<double>& values, double p) {
        if (values.empty()) return nan();

        std::vector<double> a;
        a.reserve(values.size());
        for (double v : values) a.push_back(std::abs(v));
        std::sort(a.begin(), a.end());

        const double idx = p * static_cast<double>(a.size() - 1);
        const auto lo = static_cast<std::size_t>(std::floor(idx));
        const auto hi = static_cast<std::size_t>(std::ceil(idx));
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
        double max_abs = 0.0;
        for (double v : e) {
            sum_abs += std::abs(v);
            sum_sq += v * v;
            max_abs = std::max(max_abs, std::abs(v));
        }

        s.mae = sum_abs / n;
        s.rmse = std::sqrt(sum_sq / n);
        s.max_abs = max_abs;
        s.p95_abs = percentile_abs(e, 0.95);

        double var_sum = 0.0;
        for (double v : e) var_sum += (v - s.bias) * (v - s.bias);
        s.stddev = std::sqrt(var_sum / n);
        return s;
    }

    static SignalStats signal_stats(const std::vector<double>& values) {
        SignalStats s;
        if (values.empty()) return s;

        const double n = static_cast<double>(values.size());
        s.mean = std::accumulate(values.begin(), values.end(), 0.0) / n;

        double sum_sq = 0.0;
        double max_abs = 0.0;
        for (double v : values) {
            sum_sq += v * v;
            max_abs = std::max(max_abs, std::abs(v));
        }
        s.rms = std::sqrt(sum_sq / n);
        s.max_abs = max_abs;
        s.p95_abs = percentile_abs(values, 0.95);

        double var_sum = 0.0;
        for (double v : values) var_sum += (v - s.mean) * (v - s.mean);
        s.stddev = std::sqrt(var_sum / n);
        return s;
    }

    void active_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        const auto now = this->now();
        if (msg->data && !control_active_) {
            control_active_ = true;
            control_active_since_ = now;
            RCLCPP_INFO(
                get_logger(),
                "ADRC CONTROL active. Waiting %.1f s settle time%s...",
                settle_sec_,
                require_target_ ? " and trajectory target" : "");
        } else if (!msg->data && control_active_) {
            control_active_ = false;
            measuring_ = false;
            RCLCPP_WARN(get_logger(), "ADRC left CONTROL state; measurement paused.");
        }
    }

    void state_cb(const slung_ctrl::msg::SwingADRCState::SharedPtr msg) {
        latest_state_ = *msg;
        state_rx_time_ = this->now();
        have_state_ = true;
    }

    void target_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
        target_north_ = msg->x;
        target_east_ = msg->y;
        have_target_ = true;
    }

    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        actual_north_ = msg->position[0];
        actual_east_ = msg->position[1];
        have_odom_ = true;
    }

    void truth_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        if (finished_ || !control_active_ || !have_state_) return;
        if (require_target_ && !have_target_) return;

        const auto now = this->now();
        if ((now - control_active_since_).seconds() < settle_sec_) return;

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
                "[%s] recording STARTED for %.1f s",
                experiment_name_.c_str(),
                duration_sec_);
        }

        const double elapsed = (now - measure_start_).seconds();
        if (elapsed > duration_sec_) {
            finish();
            return;
        }

        const double truth_e = msg->vector.x;
        const double truth_n = msg->vector.y;
        const double est_e = latest_state_.theta_east_deg;
        const double est_n = latest_state_.theta_north_deg;
        const double err_e = est_e - truth_e;
        const double err_n = est_n - truth_n;
        const double truth_h = std::hypot(truth_e, truth_n);
        const double err_h = std::hypot(err_e, err_n);

        err_east_.push_back(err_e);
        err_north_.push_back(err_n);
        err_horizontal_.push_back(err_h);
        truth_east_.push_back(truth_e);
        truth_north_.push_back(truth_n);
        truth_horizontal_.push_back(truth_h);
        state_age_ms_.push_back(state_age * 1000.0);

        double pos_err_e = nan();
        double pos_err_n = nan();
        double pos_err_h = nan();
        if (have_target_ && have_odom_) {
            pos_err_e = target_east_ - actual_east_;
            pos_err_n = target_north_ - actual_north_;
            pos_err_h = std::hypot(pos_err_e, pos_err_n);
            pos_err_east_.push_back(pos_err_e);
            pos_err_north_.push_back(pos_err_n);
            pos_err_horizontal_.push_back(pos_err_h);
        }

        if (csv_.is_open()) {
            csv_ << std::fixed << std::setprecision(9)
                 << elapsed << ','
                 << truth_e << ',' << est_e << ',' << err_e << ','
                 << truth_n << ',' << est_n << ',' << err_n << ','
                 << truth_h << ',' << err_h << ','
                 << (have_target_ ? target_east_ : nan()) << ','
                 << (have_odom_ ? actual_east_ : nan()) << ','
                 << pos_err_e << ','
                 << (have_target_ ? target_north_ : nan()) << ','
                 << (have_odom_ ? actual_north_ : nan()) << ','
                 << pos_err_n << ','
                 << pos_err_h << ','
                 << state_age * 1000.0 << '\n';
        }
    }

    void finish() {
        if (finished_) return;
        finished_ = true;
        measuring_ = false;
        if (csv_.is_open()) csv_.flush();

        const auto est_e = error_stats(err_east_);
        const auto est_n = error_stats(err_north_);
        const auto est_h = signal_stats(err_horizontal_);

        const auto truth_e = signal_stats(truth_east_);
        const auto truth_n = signal_stats(truth_north_);
        const auto truth_h = signal_stats(truth_horizontal_);

        const auto pos_e = error_stats(pos_err_east_);
        const auto pos_n = error_stats(pos_err_north_);
        const auto pos_h = signal_stats(pos_err_horizontal_);
        const auto state_age = signal_stats(state_age_ms_);

        RCLCPP_INFO(get_logger(), "============================================================");
        RCLCPP_INFO(get_logger(), "[%s] EVALUATION COMPLETE", experiment_name_.c_str());
        RCLCPP_INFO(get_logger(), "Samples: %zu | stale state skipped: %llu",
                    err_east_.size(),
                    static_cast<unsigned long long>(skipped_stale_state_));

        RCLCPP_INFO(get_logger(),
            "Estimation East : RMSE %.5f deg | MAE %.5f | P95 %.5f | Max %.5f | Bias %.5f",
            est_e.rmse, est_e.mae, est_e.p95_abs, est_e.max_abs, est_e.bias);
        RCLCPP_INFO(get_logger(),
            "Estimation North: RMSE %.5f deg | MAE %.5f | P95 %.5f | Max %.5f | Bias %.5f",
            est_n.rmse, est_n.mae, est_n.p95_abs, est_n.max_abs, est_n.bias);
        RCLCPP_INFO(get_logger(),
            "Estimation horiz: RMS %.5f deg | P95 %.5f | Max %.5f",
            est_h.rms, est_h.p95_abs, est_h.max_abs);

        RCLCPP_INFO(get_logger(),
            "True swing East : RMS %.5f deg | P95 %.5f | Max %.5f",
            truth_e.rms, truth_e.p95_abs, truth_e.max_abs);
        RCLCPP_INFO(get_logger(),
            "True swing North: RMS %.5f deg | P95 %.5f | Max %.5f",
            truth_n.rms, truth_n.p95_abs, truth_n.max_abs);
        RCLCPP_INFO(get_logger(),
            "True swing horiz: RMS %.5f deg | P95 %.5f | Max %.5f",
            truth_h.rms, truth_h.p95_abs, truth_h.max_abs);

        if (!pos_err_horizontal_.empty()) {
            RCLCPP_INFO(get_logger(),
                "Position East : RMSE %.5f m | P95 %.5f | Max %.5f",
                pos_e.rmse, pos_e.p95_abs, pos_e.max_abs);
            RCLCPP_INFO(get_logger(),
                "Position North: RMSE %.5f m | P95 %.5f | Max %.5f",
                pos_n.rmse, pos_n.p95_abs, pos_n.max_abs);
            RCLCPP_INFO(get_logger(),
                "Position horiz: RMS %.5f m | P95 %.5f | Max %.5f",
                pos_h.rms, pos_h.p95_abs, pos_h.max_abs);
        } else {
            RCLCPP_WARN(get_logger(), "No target/odometry overlap; position metrics unavailable.");
        }

        RCLCPP_INFO(get_logger(),
            "State age: mean %.3f ms | P95 %.3f | Max %.3f",
            state_age.mean, state_age.p95_abs, state_age.max_abs);
        RCLCPP_INFO(get_logger(), "CSV: %s", output_csv_.c_str());
        RCLCPP_INFO(get_logger(), "============================================================");

        if (auto_shutdown_) {
            rclcpp::shutdown();
        }
    }

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

    std::string experiment_name_;
    double duration_sec_ = 35.0;
    double settle_sec_ = 3.0;
    double max_state_age_sec_ = 0.020;
    bool require_target_ = true;
    std::string output_csv_;
    bool auto_shutdown_ = true;
    std::uint64_t skipped_stale_state_ = 0;

    std::vector<double> err_east_;
    std::vector<double> err_north_;
    std::vector<double> err_horizontal_;
    std::vector<double> truth_east_;
    std::vector<double> truth_north_;
    std::vector<double> truth_horizontal_;
    std::vector<double> pos_err_east_;
    std::vector<double> pos_err_north_;
    std::vector<double> pos_err_horizontal_;
    std::vector<double> state_age_ms_;

    std::ofstream csv_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_active_;
    rclcpp::Subscription<slung_ctrl::msg::SwingADRCState>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_truth_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingErrorEval>());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
