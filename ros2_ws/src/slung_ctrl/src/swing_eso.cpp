// ==============================================================================
// @File     : swing_eso.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 线性扩张状态观测器 (LESO) 节点，用于估计摆角、角速度以及系统集总扰动
// 
// @Main Work: 
//   * 节点注册：创建名为 `swing_eso` 的 ROS 2 节点
//   * 数据输入：订阅视觉测量的摆角以及高频机体加速度
//   * 核心处理：基于吊挂单摆模型，将空气阻力、未建模动态等未知因素定义为集总扰动，并扩张为新的状态变量。
//             通过配置观测器带宽 (`omega_o`) 设置反馈增益，利用视觉测量误差在 200Hz 循环中实时逼近真实状态。支持通过 ROS 2 动态参数实时调整带宽和绳长
//   * 数据输出：发布 ESO 估计的摆角、角速度，以及关键的集总扰动估计值 `/uav/swing/eso/disturbance`
// ==============================================================================

// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <mutex>
#include <algorithm>

using namespace std::chrono_literals;

// === [ Node Definition ] ===
class SwingAngleESO : public rclcpp::Node {
public:
    SwingAngleESO() : Node("swing_eso") {
        // === [ Parameters ] ===
        this->declare_parameter("g", 9.81);
        this->declare_parameter("l", 1.0);
        this->declare_parameter("omega_o", 8.0);
        this->declare_parameter("dt", 0.005);

        this->get_parameter("g", g_);
        this->get_parameter("l", l_);
        this->get_parameter("omega_o", omega_o_);
        this->get_parameter("dt", dt_);

        update_gains();

        param_sub_ = this->add_on_set_parameters_callback(
            std::bind(&SwingAngleESO::parameters_callback, this, std::placeholders::_1));

        // === [ Publishers & Subscriptions ] ===
        sub_vision_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/vision/angle", 10,
            std::bind(&SwingAngleESO::vision_callback, this, std::placeholders::_1));

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(),
            std::bind(&SwingAngleESO::accel_callback, this, std::placeholders::_1));

        pub_eso_angle_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/eso/angle", 10);
        pub_eso_rate_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/eso/rate", 10);
        pub_eso_dist_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/eso/disturbance", 10);

        // === [ Timer ] ===
        auto timer_period = std::chrono::duration<double>(dt_);
        timer_200hz_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period), 
            std::bind(&SwingAngleESO::eso_step_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "SwingAngleESO Started. Bandwidth w_o = %.1f", omega_o_);
    }

private:
    // === [ Callbacks & Computation ] ===
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (const auto &param : parameters) {
            if (param.get_name() == "g") {
                if (param.as_double() <= 0.0) { result.successful = false; result.reason = "Invalid g"; return result; }
                g_ = param.as_double();
            } else if (param.get_name() == "l") {
                if (param.as_double() <= 0.01) { result.successful = false; result.reason = "Invalid l"; return result; }
                l_ = param.as_double();
            } else if (param.get_name() == "omega_o") {
                if (param.as_double() <= 0.0) { result.successful = false; result.reason = "Invalid omega_o"; return result; }
                omega_o_ = param.as_double();
            }
        }
        
        update_gains();
        RCLCPP_INFO(this->get_logger(), "ESO parameters updated. w_o = %.1f, l = %.1f", omega_o_, l_);
        return result;
    }

    void update_gains() {
        b0_ = -1.0 / l_;
        beta1_ = 3.0 * omega_o_;
        beta2_ = 3.0 * std::pow(omega_o_, 2) - (g_ / l_);
        beta3_ = std::pow(omega_o_, 3);
    }

    void vision_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        y_x_ = msg->vector.x * M_PI / 180.0;
        y_y_ = msg->vector.y * M_PI / 180.0;
        has_vision_ = true;
    }

    void accel_callback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        a_x_ = msg->ay;  // visual world x = East, PX4 ay = East
        a_y_ = msg->ax;  // visual world y = North, PX4 ax = North
    }

    void eso_step_callback() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // RK4 with the latest received acceleration and visual sample held
        // throughout this step. No acquisition-time alignment or prediction.
        const auto integrate_axis = [&](LesoState &state, double accel, double y) {
            const auto derivative = [&](const LesoState &x) {
                const double error = has_vision_ ? y - x.x1 : 0.0;
                return LesoState{
                    x.x2 + beta1_ * error,
                    -(g_ / l_) * x.x1 + b0_ * accel + x.x3 + beta2_ * error,
                    beta3_ * error};
            };
            const auto shifted = [](const LesoState &x, const LesoState &k, double h) {
                return LesoState{x.x1 + h * k.x1, x.x2 + h * k.x2, x.x3 + h * k.x3};
            };
            const auto k1 = derivative(state);
            const auto k2 = derivative(shifted(state, k1, dt_ / 2));
            const auto k3 = derivative(shifted(state, k2, dt_ / 2));
            const auto k4 = derivative(shifted(state, k3, dt_));
            state.x1 += dt_ / 6 * (k1.x1 + 2 * k2.x1 + 2 * k3.x1 + k4.x1);
            state.x2 += dt_ / 6 * (k1.x2 + 2 * k2.x2 + 2 * k3.x2 + k4.x2);
            state.x3 += dt_ / 6 * (k1.x3 + 2 * k2.x3 + 2 * k3.x3 + k4.x3);
        };
        integrate_axis(state_x_, a_x_, y_x_);
        integrate_axis(state_y_, a_y_, y_y_);

        // === [ Data Publishing ] ===
        auto stamp = this->get_clock()->now();

        geometry_msgs::msg::Vector3Stamped msg_angle;
        msg_angle.header.stamp = stamp;
        msg_angle.header.frame_id = "world";
        msg_angle.vector.x = state_x_.x1 * 180.0 / M_PI;
        msg_angle.vector.y = state_y_.x1 * 180.0 / M_PI;
        pub_eso_angle_->publish(msg_angle);

        geometry_msgs::msg::Vector3Stamped msg_rate;
        msg_rate.header.stamp = stamp;
        msg_rate.header.frame_id = "world";
        msg_rate.vector.x = state_x_.x2 * 180.0 / M_PI;
        msg_rate.vector.y = state_y_.x2 * 180.0 / M_PI;
        pub_eso_rate_->publish(msg_rate);

        geometry_msgs::msg::Vector3Stamped msg_dist;
        msg_dist.header.stamp = stamp;
        msg_dist.header.frame_id = "world";
        msg_dist.vector.x = state_x_.x3;
        msg_dist.vector.y = state_y_.x3;
        pub_eso_dist_->publish(msg_dist);
    }

    // === [ Member Variables ] ===
    struct LesoState { double x1 = 0.0, x2 = 0.0, x3 = 0.0; };
    LesoState state_x_; 
    LesoState state_y_; 

    double a_x_ = 0.0, a_y_ = 0.0;     
    double y_x_ = 0.0, y_y_ = 0.0;     
    rclcpp::Time last_vision_time_;    
    bool has_vision_ = false;

    std::mutex data_mutex_;

    double g_, l_, b0_, omega_o_;
    double beta1_, beta2_, beta3_;
    double dt_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_eso_angle_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_eso_rate_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_eso_dist_;
    rclcpp::TimerBase::SharedPtr timer_200hz_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_sub_;
};

// === [ Main Function ] ===
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingAngleESO>());
    rclcpp::shutdown();
    return 0;
}