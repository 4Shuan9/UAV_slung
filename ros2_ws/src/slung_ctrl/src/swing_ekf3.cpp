// ==============================================================================
// @File  : swing_ekf3.cpp
// @Brief : 三状态 EKF + 视觉历史校正/重传播；East/North 原子状态输出。
//          Nominal input is measured acceleration, never acceleration setpoint.
// ==============================================================================
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "slung_ctrl/msg/swing_adrc_state.hpp"
#include "slung_ctrl/ekf3.hpp"
#include <memory>
#include <string>

class SwingAngleAugmentedEKF : public rclcpp::Node {
public:
    SwingAngleAugmentedEKF() : Node("swing_ekf3") {
        // === [ Parameters: fixed for one replay epoch ] ===
        slung_ctrl::EKFParameters p;
        p.g = declare_parameter("g", 9.81);
        p.l = declare_parameter("l", 1.0);
        p.q_omega = declare_parameter("q_omega", 0.01);
        p.q_f = declare_parameter("q_f", 0.05);
        p.max_step = declare_parameter("max_step", 0.005);
        p.initial_rate_std = declare_parameter("initial_rate_std", 0.5);
        p.initial_disturbance_std = declare_parameter("initial_disturbance_std", 0.5);
        p.nis_threshold = declare_parameter("nis_threshold", 9.0);
        p.history_sec = declare_parameter("history_sec", 1.0);
        const auto max_events = declare_parameter("max_events", 2048);
        if (max_events < 4) throw std::invalid_argument("max_events must be >= 4");
        p.max_events = static_cast<std::size_t>(max_events);
        p.validate();
        filter_ = std::make_unique<slung_ctrl::DelayedSwingEKF>(p);
        vision_std_ = declare_parameter("vision_std_deg", 0.5) * M_PI / 180.0;
        time_std_ = declare_parameter("vision_time_std", 0.002);
        max_vision_age_ = declare_parameter("max_vision_age", 0.25);
        max_input_age_ = declare_parameter("max_input_age", 0.05);
        input_future_tolerance_ = declare_parameter("input_future_tolerance", 0.02);
        max_delay_ = declare_parameter("max_visual_delay", 0.5);
        input_delay_ = declare_parameter("input_delay", 0.0);
        bias_east_ = declare_parameter("accel_bias_east", 0.0);
        bias_north_ = declare_parameter("accel_bias_north", 0.0);
        input_time_mode_ = declare_parameter("input_time_mode", std::string("offset"));
        const double publish_hz = declare_parameter("publish_hz", 200.0);
        const auto vision_topic = declare_parameter("vision_topic",
            std::string("/uav/swing/vision/angle"));
        for (double value : {vision_std_, time_std_, max_vision_age_, max_input_age_,
                             max_delay_, input_future_tolerance_, input_delay_, bias_east_, bias_north_, publish_hz}) {
            if (!std::isfinite(value)) throw std::invalid_argument("Non-finite node parameter");
        }
        if (vision_std_ <= 0 || time_std_ < 0 ||
            max_vision_age_ <= 0 || max_input_age_ <= 0 || input_future_tolerance_ < 0 ||
            input_future_tolerance_ > max_input_age_ || max_delay_ <= 0 ||
            max_delay_ >= p.history_sec || input_delay_ < 0 || publish_hz <= 0 || publish_hz > 1000 ||
            (input_time_mode_ != "offset" && input_time_mode_ != "synchronized")) {
            throw std::invalid_argument("Invalid timing/measurement parameters");
        }

        // === [ Sub/Pub ] ===
        sub_accel_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&SwingAngleAugmentedEKF::accel_callback, this, std::placeholders::_1));
        sub_vision_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(vision_topic, 20,
            std::bind(&SwingAngleAugmentedEKF::vision_callback, this, std::placeholders::_1));
        pub_state_ = create_publisher<slung_ctrl::msg::SwingADRCState>("/uav/swing/ekf3/state", 10);
        pub_angle_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/ekf3/angle", 10);
        pub_rate_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/ekf3/rate", 10);
        pub_dist_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/ekf3/disturbance", 10);
        pub_cov_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/swing/ekf3/covariance", 10);
        pub_diag_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/swing/ekf3/diagnostics", 10);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz),
            std::bind(&SwingAngleAugmentedEKF::publish_estimate, this));
        param_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = "EKF replay parameters are fixed per run; restart with new parameters";
            return result;
        });
        RCLCPP_INFO(get_logger(), "Augmented EKF ready: two 3-state axes, replay %.2fs, input clock=%s",
                    p.history_sec, input_time_mode_.c_str());
    }

private:
    bool clock_ready(double t) {
        if (t <= 0) return false;
        if (last_clock_ >= 0 && t < last_clock_ - 0.001) {
            reset("ROS clock moved backwards");
        }
        last_clock_ = t;
        return true;
    }
    void reset(const char *reason) {
        filter_->reset();
        offset_set_ = false;
        last_px4_stamp_ = 0;
        last_input_ = last_published_ = -1;
        RCLCPP_WARN(get_logger(), "EKF reset: %s; waiting for input + a new visual frame", reason);
    }
    void accel_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        const double t = now().seconds();
        if (!clock_ready(t) || !std::isfinite(msg->ax) || !std::isfinite(msg->ay) ||
            msg->timestamp_sample == 0) return;
        // DDS preserves ordering for this writer. A backward PX4 sample means restart.
        if (last_px4_stamp_ && msg->timestamp_sample < last_px4_stamp_) reset("PX4 sample clock restarted");
        last_px4_stamp_ = msg->timestamp_sample;
        if (!offset_set_) {
            // Constant anchor, NOT re-estimated every callback. Transport delay remains a
            // calibration uncertainty; this does not claim exact PX4/ROS synchronization.
            input_offset_ = input_time_mode_ == "offset" ? t - msg->timestamp_sample * 1e-6 - input_delay_ : -input_delay_;
            offset_set_ = true;
        }
        const double sample_time = msg->timestamp_sample * 1e-6 + input_offset_;
        if (sample_time > t + input_future_tolerance_ || t - sample_time > max_input_age_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Input timestamp age %.4fs invalid; check PX4/ROS physical clocks", t - sample_time);
            return;
        }
        try {
            // PX4 NED ax=North, ay=East; visual world x=East, y=North.
            const auto result = filter_->add_input(sample_time, msg->ay - bias_east_, msg->ax - bias_north_);
            if (result == slung_ctrl::EventResult::accepted) last_input_ = std::max(last_input_, sample_time);
        } catch (const std::exception &e) { reset(e.what()); }
    }
    void vision_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        const double t = now().seconds();
        if (!clock_ready(t)) return;
        const double capture = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()).seconds();
        const double east = msg->vector.x;
        const double north = msg->vector.y;
        if (!std::isfinite(east) || !std::isfinite(north) || capture <= 0 || capture > t ||
            t - capture > max_delay_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Visual frame rejected: time/domain invalid");
            return;
        }
        last_delay_ = t - capture;
        try {
            const auto result = filter_->add_vision(capture, east, north,
                vision_std_ * vision_std_, vision_std_ * vision_std_, time_std_);
            if (result == slung_ctrl::EventResult::too_old || result == slung_ctrl::EventResult::no_input) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Visual frame has no preceding input/checkpoint; skipped");
            }
        } catch (const std::exception &e) { reset(e.what()); }
    }
    void publish_estimate() {
        const auto stamp = now();
        const double t = stamp.seconds();
        if (!clock_ready(t) || !filter_->initialized() || t <= last_published_ ||
            last_input_ < 0 || t < last_input_ || t - last_input_ > max_input_age_) return;
        try {
            const auto s = filter_->estimate_at(t);
            std_msgs::msg::Float64MultiArray diag;
            diag.data = {t - s.last_vision[0], t - s.last_vision[1], t - last_input_,
                static_cast<double>(filter_->event_count()), s.nis[0], s.nis[1], last_delay_};
            pub_diag_->publish(diag);
            for (int i = 0; i < 2; ++i) {
                if (t - s.last_vision[i] > max_vision_age_) return;
            }
            slung_ctrl::msg::SwingADRCState state;
            state.header.stamp = stamp; state.header.frame_id = "world";
            state.theta_east_rad = s.axis[0].x(0);
            state.theta_north_rad = s.axis[1].x(0);
            state.rate_east_rad_s = s.axis[0].x(1);
            state.rate_north_rad_s = s.axis[1].x(1);
            state.disturbance_east_rad_s2 = s.axis[0].x(2);
            state.disturbance_north_rad_s2 = s.axis[1].x(2);
            pub_state_->publish(state);
            geometry_msgs::msg::Vector3Stamped angle, rate, dist;
            angle.header = rate.header = dist.header = state.header;
            angle.vector.x = state.theta_east_rad; angle.vector.y = state.theta_north_rad;
            rate.vector.x = state.rate_east_rad_s; rate.vector.y = state.rate_north_rad_s;
            dist.vector.x = state.disturbance_east_rad_s2; dist.vector.y = state.disturbance_north_rad_s2;
            pub_angle_->publish(angle); pub_rate_->publish(rate); pub_dist_->publish(dist);
            std_msgs::msg::Float64MultiArray covariance;
            for (const auto &axis : s.axis) {
                for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col) {
                    covariance.data.push_back(axis.P(row, col));
                }
            }
            pub_cov_->publish(covariance);
            last_published_ = t;
        } catch (const std::exception &e) { reset(e.what()); }
    }

    std::unique_ptr<slung_ctrl::DelayedSwingEKF> filter_;
    double vision_std_, time_std_, max_vision_age_, max_input_age_, max_delay_;
    double input_future_tolerance_, input_delay_, bias_east_, bias_north_, input_offset_ = 0;
    double last_clock_ = -1, last_input_ = -1, last_published_ = -1, last_delay_ = 0;
    uint64_t last_px4_stamp_ = 0;
    bool offset_set_ = false;
    std::string input_time_mode_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_accel_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_;
    rclcpp::Publisher<slung_ctrl::msg::SwingADRCState>::SharedPtr pub_state_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angle_, pub_rate_, pub_dist_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cov_, pub_diag_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_guard_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<SwingAngleAugmentedEKF>()); }
    catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("swing_ekf3"), "%s", e.what()); rclcpp::shutdown(); return 1; }
    rclcpp::shutdown();
    return 0;
}
