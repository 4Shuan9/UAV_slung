// ==============================================================================
// @File     : swing_ekf2.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 扩展卡尔曼滤波 (EKF) 摆角估计节点，用于融合单摆动力学模型与视觉观测
// 
// @Main Work: 
//   * 节点注册：创建名为 `swing_ekf2` 的 ROS 2 节点
//   * 数据输入：订阅含有噪声/延迟的视觉摆角测量值 `/uav/swing/vision/...` 和机体加速度
//   * 核心处理：定义非线性单摆状态方程（状态变量为摆角与角速度）。在 200Hz 预测步骤中利用机体加速度计算状态预测与协方差矩阵；
//             在视觉数据到达的更新步骤中，计算雅可比矩阵、卡尔曼增益及残差，进行状态的最优后验修正
//   * 数据输出：发布 EKF 滤波后的最优摆角 `/uav/swing/ekf2/angle` 和角速度估计值 `/uav/swing/ekf2/rate`
// ==============================================================================

// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

class SwingAngleEKF : public rclcpp::Node {
public:
    SwingAngleEKF() : Node("swing_ekf2") {
        // === [ Parameters ] ===
        this->declare_parameter("g", 9.81);
        this->declare_parameter("l", 1.0);
        this->declare_parameter("damping_c", 0.2); // 等效阻尼系数
        this->declare_parameter("dt", 0.005);      // 200Hz 预测步长

        g_ = this->get_parameter("g").as_double();
        l_ = this->get_parameter("l").as_double();
        c_ = this->get_parameter("damping_c").as_double();
        dt_ = this->get_parameter("dt").as_double();

        // === [ EKF 初始化 ] ===
        // 状态向量 X = [theta, theta_dot]^T
        X_x_ = Eigen::Vector2d::Zero();
        X_y_ = Eigen::Vector2d::Zero();

        // 协方差矩阵 P (初始不确定度)
        P_x_ = Eigen::Matrix2d::Identity() * 0.1;
        P_y_ = Eigen::Matrix2d::Identity() * 0.1;

        // 过程噪声协方差 Q (动力学模型的不确定度，可微调)
        Q_ = Eigen::Matrix2d::Zero();
        Q_(0, 0) = 0.001;  // 角度模型噪声
        Q_(1, 1) = 0.1;    // 角速度模型噪声

        // 观测噪声协方差 R (视觉传感器的像素抖动噪声，可微调)
        R_ = 0.1;

        // 观测矩阵 H (只能测到角度 theta)
        H_ = Eigen::RowVector2d(1.0, 0.0);

        // === [ Sub/Pub ] ===
        sub_vision_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/swing/vision/angle", 10,
            std::bind(&SwingAngleEKF::vision_callback, this, std::placeholders::_1));

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(),
            std::bind(&SwingAngleEKF::accel_callback, this, std::placeholders::_1));

        pub_ekf_angle_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/ekf2/angle", 10);
        pub_ekf_rate_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/ekf2/rate", 10);

        // === [ Timer ] ===
        timer_200hz_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt_)), 
            std::bind(&SwingAngleEKF::ekf_predict_step, this));
            
        RCLCPP_INFO(this->get_logger(), "SwingAngle EKF Benchmark Node Started.");
    }

private:
    // ================== [ EKF 步骤 1：预测 (Predict) ] ==================
    void ekf_predict_step() {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // X轴预测
        predict_axis(X_x_, P_x_, a_x_);
        // Y轴预测
        predict_axis(X_y_, P_y_, a_y_);

        // === [ 发布当前时刻的估计值 ] ===
        auto stamp = this->get_clock()->now();
        geometry_msgs::msg::Vector3Stamped msg_angle, msg_rate;
        msg_angle.header.stamp = msg_rate.header.stamp = stamp;
        msg_angle.header.frame_id = msg_rate.header.frame_id = "world";

        msg_angle.vector.x = X_x_(0) * 180.0 / M_PI; 
        msg_angle.vector.y = X_y_(0) * 180.0 / M_PI;
        pub_ekf_angle_->publish(msg_angle);

        msg_rate.vector.x = X_x_(1) * 180.0 / M_PI; 
        msg_rate.vector.y = X_y_(1) * 180.0 / M_PI;
        pub_ekf_rate_->publish(msg_rate);
    }

    void predict_axis(Eigen::Vector2d& X, Eigen::Matrix2d& P, double accel) {
        double theta = X(0);
        double theta_dot = X(1);

        // 1. 状态预测 (使用非线性物理模型)
        double theta_ddot = -(g_ / l_) * std::sin(theta) - (accel / l_) * std::cos(theta) - c_ * theta_dot;
        
        X(0) = theta + theta_dot * dt_;
        X(1) = theta_dot + theta_ddot * dt_;

        // 2. 计算雅可比矩阵 F (对非线性模型求偏导)
        Eigen::Matrix2d F;
        F(0, 0) = 1.0;
        F(0, 1) = dt_;
        // 对 theta 求偏导
        F(1, 0) = ( -(g_ / l_) * std::cos(theta) + (accel / l_) * std::sin(theta) ) * dt_; 
        // 对 theta_dot 求偏导
        F(1, 1) = 1.0 - c_ * dt_;

        // 3. 协方差预测 P = F*P*F^T + Q
        P = F * P * F.transpose() + Q_;
    }

    // ================== [ EKF 步骤 2：更新 (Update) ] ==================
    void vision_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 视觉测量值转换为弧度
        double z_x = msg->vector.x * M_PI / 180.0;
        double z_y = msg->vector.y * M_PI / 180.0;

        // X轴更新
        update_axis(X_x_, P_x_, z_x);
        // Y轴更新
        update_axis(X_y_, P_y_, z_y);
    }

    void update_axis(Eigen::Vector2d& X, Eigen::Matrix2d& P, double z) {
        // 1. 计算残差 (Innovation)
        double y_residual = z - H_ * X;

        // 2. 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
        double S = H_ * P * H_.transpose() + R_;
        Eigen::Vector2d K = P * H_.transpose() / S;

        // 3. 状态更新
        X = X + K * y_residual;

        // 4. 协方差更新 P = (I - K * H) * P
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        P = (I - K * H_) * P;
    }

    void accel_callback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        a_x_ = msg->ay;  // visual world x = East, PX4 ay = East
        a_y_ = msg->ax;  // visual world y = North, PX4 ax = North
    }

    // === [ 变量区 ] ===
    double g_, l_, c_, dt_;
    double a_x_ = 0.0, a_y_ = 0.0;     
    std::mutex data_mutex_;

    Eigen::Vector2d X_x_, X_y_;
    Eigen::Matrix2d P_x_, P_y_;
    Eigen::Matrix2d Q_;
    double R_;
    Eigen::RowVector2d H_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ekf_angle_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ekf_rate_;
    rclcpp::TimerBase::SharedPtr timer_200hz_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingAngleEKF>());
    rclcpp::shutdown();
    return 0;
}