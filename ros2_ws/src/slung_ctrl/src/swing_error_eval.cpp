// ==============================================================================
// @File  : swing_error_eval.cpp
// @Brief : One-shot quantitative baseline evaluator for ESO vs Gazebo truth.
//
// The evaluator deliberately ignores the ground/takeoff period.  Sampling starts
// only when /uav/adrc_debug/control_active == true, then waits settle_sec, and
// also requires a fresh synchronized LESO state.  Therefore the initial period
// in which the red payload is not yet visible is never included in the baseline.
//
// Error convention: estimate - truth [deg]
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "slung_ctrl/msg/swing_adrc_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

class SwingErrorEval : public rclcpp::Node {
public:
    SwingErrorEval() : Node("swing_error_eval") {
        declare_parameter("duration_sec", 30.0);
        declare_parameter("settle_sec", 5.0);
        declare_parameter("max_state_age_ms", 20.0);
        declare_parameter("output_csv", std::string("/tmp/swing_error_baseline.csv"));
        declare_parameter("auto_shutdown", true);

        duration_sec_ = get_parameter("duration_sec").as_double();
        settle_sec_ = get_parameter("settle_sec").as_double();
        max_state_age_sec_ = get_parameter("max_state_age_ms").as_double() / 1000.0;
        output_csv_ = get_parameter("output_csv").as_string();
        auto_shutdown_ = get_parameter("auto_shutdown").as_bool();

        sub_active_ = create_subscription<std_msgs::msg::Bool>(
            "/uav/adrc_debug/control_active", 10,
            std::bind(&SwingErrorEval::active_cb, this, std::placeholders::_1));

        sub_state_ = create_subscription<slung_ctrl::msg::SwingADRCState>(
            "/uav/eso_fusion/adrc_state", 10,
            std::bind(&SwingErrorEval::state_cb, this, std::placeholders::_1));

        sub_truth_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/truth/swing_angle/world_frame/cartesian", 10,
            std::bind(&SwingErrorEval::truth_cb, this, std::placeholders::_1));

        csv_.open(output_csv_, std::ios::out | std::ios::trunc);
        if (!csv_.is_open()) {
            RCLCPP_WARN(get_logger(), "Could not open CSV: %s (terminal summary will still work)", output_csv_.c_str());
        } else {
            csv_ << "elapsed_sec,truth_east_deg,est_east_deg,error_east_deg,"
                    "truth_north_deg,est_north_deg,error_north_deg,"
                    "f_east_rad_s2,f_north_rad_s2,state_age_ms\n";
        }

        RCLCPP_INFO(get_logger(),
            "Baseline evaluator armed. Waiting for ADRC CONTROL state. "
            "Then wait %.1f s and record %.1f s. max_state_age=%.1f ms",
            settle_sec_, duration_sec_, max_state_age_sec_ * 1000.0);
    }

private:
    struct SampleStats {
        double bias = 0.0;
        double mae = 0.0;
        double rmse = 0.0;
        double stddev = 0.0;
        double max_abs = 0.0;
        double p95_abs = 0.0;
    };

    struct SignalStats {
        double mean = 0.0;
        double rms = 0.0;
        double stddev = 0.0;
        double max_abs = 0.0;
        double p95_abs = 0.0;
    };

    bool control_active_ = false;
    bool have_state_ = false;
    bool measuring_ = false;
    bool finished_ = false;

    rclcpp::Time control_active_since_{0, 0, RCL_ROS_TIME};
    rclcpp::Time state_rx_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time measure_start_{0, 0, RCL_ROS_TIME};

    slung_ctrl::msg::SwingADRCState latest_state_;

    double duration_sec_ = 30.0;
    double settle_sec_ = 5.0;
    double max_state_age_sec_ = 0.020;
    std::string output_csv_;
    bool auto_shutdown_ = true;

    std::vector<double> err_east_;
    std::vector<double> err_north_;
    std::vector<double> f_east_;
    std::vector<double> f_north_;
    std::vector<double> state_age_ms_;
    uint64_t skipped_stale_state_ = 0;

    std::ofstream csv_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_active_;
    rclcpp::Subscription<slung_ctrl::msg::SwingADRCState>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_truth_;

    void active_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        const auto now = this->now();
        if (msg->data && !control_active_) {
            control_active_ = true;
            control_active_since_ = now;
            measuring_ = false;
            RCLCPP_INFO(get_logger(),
                "ADRC CONTROL active. Waiting %.1f s settling time before baseline starts...",
                settle_sec_);
        } else if (!msg->data && control_active_) {
            control_active_ = false;
            measuring_ = false;
            RCLCPP_WARN(get_logger(), "ADRC left CONTROL state. Baseline paused/reset.");
        }
    }

    void state_cb(const slung_ctrl::msg::SwingADRCState::SharedPtr msg) {
        latest_state_ = *msg;
        state_rx_time_ = this->now();
        have_state_ = true;
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
            RCLCPP_INFO(get_logger(),
                "Baseline recording STARTED. Vision + truth + synchronized LESO state are valid. Duration %.1f s.",
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
        f_east_.push_back(latest_state_.disturbance_east_rad_s2);
        f_north_.push_back(latest_state_.disturbance_north_rad_s2);
        state_age_ms_.push_back(state_age * 1000.0);

        if (csv_.is_open()) {
            csv_ << std::fixed << std::setprecision(9)
                 << elapsed << ','
                 << truth_east << ',' << est_east << ',' << err_e << ','
                 << truth_north << ',' << est_north << ',' << err_n << ','
                 << latest_state_.disturbance_east_rad_s2 << ','
                 << latest_state_.disturbance_north_rad_s2 << ','
                 << state_age * 1000.0 << '\n';
        }
    }

    static double percentile_abs(const std::vector<double>& values, double p) {
        if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
        std::vector<double> a;
        a.reserve(values.size());
        for (double v : values) a.push_back(std::abs(v));
        std::sort(a.begin(), a.end());
        const double idx = p * static_cast<double>(a.size() - 1);
        const auto lo = static_cast<size_t>(std::floor(idx));
        const auto hi = static_cast<size_t>(std::ceil(idx));
        if (lo == hi) return a[lo];
        const double w = idx - static_cast<double>(lo);
        return a[lo] * (1.0 - w) + a[hi] * w;
    }

    static SampleStats error_stats(const std::vector<double>& e) {
        SampleStats s;
        if (e.empty()) return s;
        const double n = static_cast<double>(e.size());
        s.bias = std::accumulate(e.begin(), e.end(), 0.0) / n;
        double sum_abs = 0.0, sum_sq = 0.0, var_sum = 0.0;
        for (double v : e) {
            sum_abs += std::abs(v);
            sum_sq += v * v;
            s.max_abs = std::max(s.max_abs, std::abs(v));
        }
        s.mae = sum_abs / n;
        s.rmse = std::sqrt(sum_sq / n);
        for (double v : e) var_sum += (v - s.bias) * (v - s.bias);
        s.stddev = std::sqrt(var_sum / n);
        s.p95_abs = percentile_abs(e, 0.95);
        return s;
    }

    static SignalStats signal_stats(const std::vector<double>& x) {
        SignalStats s;
        if (x.empty()) return s;
        const double n = static_cast<double>(x.size());
        s.mean = std::accumulate(x.begin(), x.end(), 0.0) / n;
        double sum_sq = 0.0, var_sum = 0.0;
        for (double v : x) {
            sum_sq += v * v;
            s.max_abs = std::max(s.max_abs, std::abs(v));
        }
        s.rms = std::sqrt(sum_sq / n);
        for (double v : x) var_sum += (v - s.mean) * (v - s.mean);
        s.stddev = std::sqrt(var_sum / n);
        s.p95_abs = percentile_abs(x, 0.95);
        return s;
    }

    void finish() {
        finished_ = true;
        measuring_ = false;
        if (csv_.is_open()) csv_.flush();

        const auto e = error_stats(err_east_);
        const auto n = error_stats(err_north_);
        const auto fe = signal_stats(f_east_);
        const auto fn = signal_stats(f_north_);
        const auto age = signal_stats(state_age_ms_);

        RCLCPP_INFO(get_logger(), "\n========== HOVER BASELINE RESULT ==========");
        RCLCPP_INFO(get_logger(), "Samples: %zu | skipped stale state: %lu", err_east_.size(), skipped_stale_state_);
        RCLCPP_INFO(get_logger(), "East error [deg] : Bias=%+.4f MAE=%.4f RMSE=%.4f Std=%.4f MaxAbs=%.4f P95Abs=%.4f",
                    e.bias, e.mae, e.rmse, e.stddev, e.max_abs, e.p95_abs);
        RCLCPP_INFO(get_logger(), "North error [deg]: Bias=%+.4f MAE=%.4f RMSE=%.4f Std=%.4f MaxAbs=%.4f P95Abs=%.4f",
                    n.bias, n.mae, n.rmse, n.stddev, n.max_abs, n.p95_abs);
        RCLCPP_INFO(get_logger(), "F East [rad/s^2] : Mean=%+.5f RMS=%.5f Std=%.5f MaxAbs=%.5f P95Abs=%.5f",
                    fe.mean, fe.rms, fe.stddev, fe.max_abs, fe.p95_abs);
        RCLCPP_INFO(get_logger(), "F North[rad/s^2] : Mean=%+.5f RMS=%.5f Std=%.5f MaxAbs=%.5f P95Abs=%.5f",
                    fn.mean, fn.rms, fn.stddev, fn.max_abs, fn.p95_abs);
        RCLCPP_INFO(get_logger(), "State age [ms]   : Mean=%.3f RMS=%.3f MaxAbs=%.3f P95Abs=%.3f",
                    age.mean, age.rms, age.max_abs, age.p95_abs);
        RCLCPP_INFO(get_logger(), "CSV: %s", output_csv_.c_str());
        RCLCPP_INFO(get_logger(), "===========================================\n");

        if (auto_shutdown_) {
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingErrorEval>());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
