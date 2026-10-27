// ==============================================================================
// @File     : swing_ekf2.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 两状态非线性 EKF 基准：摆角、角速度；无扰动增广、无时间补偿。
// Codex-modified with owner authorization, 2026-10-27: align comparable numerical
// and noise settings with EKF3. Measurements act at receipt time; capture stamps
// are deliberately not used. No history buffer, replay or delay compensation.
// ==============================================================================
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
struct AxisState2 {
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity();
};

// The theta/omega subsystem of EKF3, with no disturbance state or fixed damping.
void predict_axis(AxisState2 &s, double a, double dt, double g, double l,
                  double q_omega, double max_step) {
    if (!std::isfinite(dt) || dt < 0 || !std::isfinite(a)) {
        throw std::invalid_argument("Invalid prediction interval/input");
    }
    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    W(1, 1) = q_omega;  // continuous rad^2/s^3; no per-tick angle noise
    const auto derivative = [&](const Eigen::Vector2d &x, const Eigen::Matrix2d &P) {
        Eigen::Vector2d dx{x(1), -g / l * std::sin(x(0)) - a / l * std::cos(x(0))};
        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        A(0, 1) = 1.0;
        A(1, 0) = -g / l * std::cos(x(0)) + a / l * std::sin(x(0));
        return std::make_pair(dx, Eigen::Matrix2d(A * P + P * A.transpose() + W));
    };
    const int steps = std::max(1, static_cast<int>(std::ceil(dt / max_step)));
    const double h = dt / steps;
    for (int i = 0; i < steps; ++i) {
        const auto k1 = derivative(s.x, s.P);
        const auto k2 = derivative(s.x + h / 2 * k1.first, s.P + h / 2 * k1.second);
        const auto k3 = derivative(s.x + h / 2 * k2.first, s.P + h / 2 * k2.second);
        const auto k4 = derivative(s.x + h * k3.first, s.P + h * k3.second);
        s.x += h / 6 * (k1.first + 2 * k2.first + 2 * k3.first + k4.first);
        s.P += h / 6 * (k1.second + 2 * k2.second + 2 * k3.second + k4.second);
        s.P = (0.5 * (s.P + s.P.transpose())).eval();
    }
    if (!s.x.allFinite() || !s.P.allFinite()) throw std::runtime_error("Non-finite prediction");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(s.P);
    if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < -1e-10) {
        throw std::runtime_error("Prediction covariance lost positive semidefiniteness");
    }
}

bool correct_axis(AxisState2 &s, double y, double R, double threshold) {
    if (!std::isfinite(y) || !std::isfinite(R) || R <= 0) {
        throw std::invalid_argument("Invalid visual measurement");
    }
    const double residual = y - s.x(0);
    const double S = s.P(0, 0) + R;
    if (!std::isfinite(S) || S <= 0) throw std::runtime_error("Invalid innovation covariance");
    if (threshold > 0 && residual * residual / S > threshold) return false;
    const Eigen::Vector2d K = s.P.col(0) / S;
    Eigen::Matrix2d M = Eigen::Matrix2d::Identity();
    M.col(0) -= K;
    s.x += K * residual;
    s.P = M * s.P * M.transpose() + R * K * K.transpose();  // Joseph form, as EKF3
    s.P = (0.5 * (s.P + s.P.transpose())).eval();
    return true;
}
}  // namespace

class SwingAngleEKF : public rclcpp::Node {
public:
    SwingAngleEKF() : Node("swing_ekf2") {
        g_ = declare_parameter("g", 9.81);
        l_ = declare_parameter("l", 1.0);
        q_omega_ = declare_parameter("q_omega", 0.01);
        max_step_ = declare_parameter("max_step", 0.005);
        initial_rate_std_ = declare_parameter("initial_rate_std", 0.5);
        nis_threshold_ = declare_parameter("nis_threshold", 9.0);
        vision_std_ = declare_parameter("vision_std_deg", 0.5) * M_PI / 180.0;
        time_std_ = declare_parameter("vision_time_std", 0.002);
        const double publish_hz = declare_parameter("publish_hz", 200.0);
        const auto vision_topic = declare_parameter("vision_topic",
            std::string("/uav/swing/vision/angle"));
        for (double value : {g_, l_, q_omega_, max_step_, initial_rate_std_,
                             nis_threshold_, vision_std_, time_std_, publish_hz}) {
            if (!std::isfinite(value)) throw std::invalid_argument("Non-finite EKF parameter");
        }
        if (g_ <= 0 || l_ <= 0 || q_omega_ < 0 || max_step_ <= 0 ||
            initial_rate_std_ <= 0 || vision_std_ <= 0 || time_std_ < 0 ||
            publish_hz <= 0 || publish_hz > 1000) {
            throw std::invalid_argument("Invalid EKF parameters");
        }
        sub_accel_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&SwingAngleEKF::accel_callback, this, std::placeholders::_1));
        sub_vision_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(vision_topic, 20,
            std::bind(&SwingAngleEKF::vision_callback, this, std::placeholders::_1));
        pub_angle_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/ekf2/angle", 10);
        pub_rate_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/ekf2/rate", 10);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_hz),
            std::bind(&SwingAngleEKF::publish_estimate, this));
        param_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = false;
            result.reason = "EKF parameters are fixed per run; restart with new parameters";
            return result;
        });
        RCLCPP_INFO(get_logger(), "Two-state EKF ready: receipt-time updates, RK4; no disturbance state or delay compensation");
    }

private:
    void reset(const char *reason) {
        initialized_ = have_input_ = false;
        state_time_ = last_published_ = -1;
        accel_.fill(0.0);
        axis_ = {};
        RCLCPP_WARN(get_logger(), "EKF reset: %s; waiting for input + a new visual frame", reason);
    }
    bool clock_ready(double t) {
        if (t <= 0) return false;
        if (last_clock_ >= 0 && t < last_clock_ - 0.001) reset("ROS clock moved backwards");
        last_clock_ = t;
        return true;
    }
    void advance(double t) {
        if (!initialized_) return;
        for (int i = 0; i < 2; ++i) {
            predict_axis(axis_[i], accel_[i], t - state_time_, g_, l_, q_omega_, max_step_);
        }
        state_time_ = t;
    }
    void accel_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        const double t = now().seconds();
        if (!clock_ready(t) || !std::isfinite(msg->ax) || !std::isfinite(msg->ay)) return;
        try {
            advance(t);  // previous latest input held until this receipt, not its capture stamp
            accel_ = {{msg->ay, msg->ax}};  // East/North from PX4 NED
            have_input_ = true;
        } catch (const std::exception &e) { reset(e.what()); }
    }
    void vision_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        const double t = now().seconds();
        const std::array<double, 2> measurement{{msg->vector.x,
                                               msg->vector.y}};
        if (!clock_ready(t) || !have_input_ || !std::isfinite(measurement[0]) ||
            !std::isfinite(measurement[1])) return;
        try {
            if (!initialized_) {
                for (int i = 0; i < 2; ++i) {
                    axis_[i].x << measurement[i], 0.0;
                    axis_[i].P = Eigen::Vector2d(vision_std_ * vision_std_,
                        initial_rate_std_ * initial_rate_std_).asDiagonal();
                }
                state_time_ = t;
                initialized_ = true;
                return;
            }
            advance(t);
            for (int i = 0; i < 2; ++i) {
                // Same nominal timestamp-jitter variance as EKF3; does not remove latency.
                const double rate = axis_[i].x(1);
                const double R = vision_std_ * vision_std_ + rate * rate * time_std_ * time_std_;
                correct_axis(axis_[i], measurement[i], R, nis_threshold_);
            }
        } catch (const std::exception &e) { reset(e.what()); }
    }
    void publish_estimate() {
        const auto stamp = now();
        const double t = stamp.seconds();
        if (!clock_ready(t) || !initialized_ || t <= last_published_) return;
        try {
            advance(t);
            geometry_msgs::msg::Vector3Stamped angle, rate;
            angle.header.stamp = rate.header.stamp = stamp;
            angle.header.frame_id = rate.header.frame_id = "world";
            angle.vector.x = axis_[0].x(0);
            angle.vector.y = axis_[1].x(0);
            rate.vector.x = axis_[0].x(1);
            rate.vector.y = axis_[1].x(1);
            pub_angle_->publish(angle);
            pub_rate_->publish(rate);
            last_published_ = t;
        } catch (const std::exception &e) { reset(e.what()); }
    }

    std::array<AxisState2, 2> axis_;
    std::array<double, 2> accel_{{0.0, 0.0}};
    double g_, l_, q_omega_, max_step_, initial_rate_std_, nis_threshold_, vision_std_, time_std_;
    double state_time_ = -1, last_clock_ = -1, last_published_ = -1;
    bool initialized_ = false, have_input_ = false;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_accel_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_angle_, pub_rate_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_guard_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<SwingAngleEKF>()); }
    catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("swing_ekf2"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
