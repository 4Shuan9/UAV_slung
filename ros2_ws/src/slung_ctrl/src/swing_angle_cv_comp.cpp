// ============================================================================
// @File     : swing_angle_cv_comp.cpp
// @Author   : 4Shuan9
// @Date     : 2026-09-16
// @Brief    : 视觉摆角延时补偿节点，引入机体加速度前馈以补偿视觉传感的低频与时间延迟
//
// @Main Work:
//   * 节点注册：创建名为 `swing_angle_cv_comp` 的 ROS 2 节点
//   * 数据输入：除像素坐标和相机位姿外，额外高频订阅无人机机体加速度 `/fmu/out/vehicle_local_position`
//   * 核心处理：在两次低频视觉数据帧之间，利用单摆运动学模型与机体加速度对摆角和角速度进行高频欧拉积分预测；
//             当真实视觉数据到达时，将其作为观测值对积分预测状态进行修正与平滑
//   * 数据输出：发布经过加速度运动学补偿后的机体系/世界系摆角数据，改善纯视觉解算的迟滞效应
//
// Coordinate convention used internally:
//   Gazebo World X = East
//   Gazebo World Y = North
//   PX4 local X    = North
//   PX4 local Y    = East
//
// Important:
//   本次修改只统一 East/North 语义命名，不改变原有数值映射、算法、参数和外部 topic 协议。
// ============================================================================

// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

// === [ Node Definition ] ===
class SwingAngleCVComp : public rclcpp::Node {
public:
    SwingAngleCVComp() : Node("swing_angle_cv_comp") {
        // === [ Configuration ] ===
        const double width = 640.0;
        const double height = 480.0;
        cx_ = width / 2.0;
        cy_ = height / 2.0;
        const double hfov = 1.74;
        fx_ = cx_ / std::tan(hfov / 2.0);
        fy_ = fx_;
        nominal_rope_length_ = 1.0;

        theta_east_ = 0.0;
        theta_north_ = 0.0;
        theta_rate_east_ = 0.0;
        theta_rate_north_ = 0.0;
        is_initialized_ = false;
        last_q_ = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

        sub_pixel_pos_.subscribe(this, "/uav/cv/payload_pose");
        sub_uav_pose_.subscribe(this, "/model/x500_mono_cam_down/pose");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_pixel_pos_, sub_uav_pose_);
        sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02));
        sync_->registerCallback(
            std::bind(&SwingAngleCVComp::sync_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&SwingAngleCVComp::accel_callback, this, std::placeholders::_1));

        pub_body_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/cv_comp/swing_angle/body_frame/cartesian", 10);
        pub_body_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/cv_comp/swing_angle/body_frame/polar", 10);
        pub_world_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/cv_comp/swing_angle/world_frame/cartesian", 10);
        pub_world_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/cv_comp/swing_angle/world_frame/polar", 10);

        RCLCPP_INFO(this->get_logger(), "SwingAngleCV Compensator Started.");
    }

private:
    // === [ Callback & Computation ] ===
    void accel_callback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& msg) {
        auto now = this->get_clock()->now();
        if (last_acc_time_.nanoseconds() == 0) {
            last_acc_time_ = now;
            return;
        }

        double dt = (now - last_acc_time_).seconds();
        last_acc_time_ = now;

        if (dt > 0.1) dt = 0.01;

        // Source variables from PX4 local NED frame.
        const double accel_x_PX4 = msg->ax;  // North
        const double accel_y_PX4 = msg->ay;  // East

        // Physical-direction variables used by the pendulum model.
        const double accel_north = accel_x_PX4;
        const double accel_east  = accel_y_PX4;

        const double g = 9.81;
        const double damping = 0.2;

        // East channel: theta_east <-> accel_east
        const double theta_accel_east =
            -(g / nominal_rope_length_) * std::sin(theta_east_)
            - (accel_east / nominal_rope_length_) * std::cos(theta_east_)
            - damping * theta_rate_east_;

        // North channel: theta_north <-> accel_north
        const double theta_accel_north =
            -(g / nominal_rope_length_) * std::sin(theta_north_)
            - (accel_north / nominal_rope_length_) * std::cos(theta_north_)
            - damping * theta_rate_north_;

        theta_rate_east_ += theta_accel_east * dt;
        theta_rate_north_ += theta_accel_north * dt;

        theta_east_ += theta_rate_east_ * dt;
        theta_north_ += theta_rate_north_ * dt;

        if (is_initialized_) {
            publish_compensated_data(now);
        }
    }

    void sync_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& pixel_msg,
                       const geometry_msgs::msg::PoseStamped::ConstSharedPtr& uav_msg) {

        const double u = pixel_msg->point.x;
        const double v = pixel_msg->point.y;
        const double x_c = (u - cx_) / fx_;
        const double y_c = (v - cy_) / fy_;

        // Camera/body-frame mapping. Body-frame variables are intentionally not
        // renamed to East/North because the body frame rotates with vehicle yaw.
        const double xb = -y_c;
        const double yb = -x_c;
        const double zb = -1.0;
        Eigen::Vector3d V_body(xb, yb, zb);

        Eigen::Quaterniond q(
            uav_msg->pose.orientation.w,
            uav_msg->pose.orientation.x,
            uav_msg->pose.orientation.y,
            uav_msg->pose.orientation.z);

        if (q.norm() < 1e-5) return;
        q.normalize();
        last_q_ = q;

        Eigen::Vector3d V_world = q * V_body;
        const double xw = V_world.x();  // Gazebo World X -> East
        const double yw = V_world.y();  // Gazebo World Y -> North
        const double zw = V_world.z();

        // Source variables from Gazebo world frame.
        const double theta_x_W_Gazebo = std::atan2(xw, -zw);
        const double theta_y_W_Gazebo = std::atan2(yw, -zw);

        // Physical directions.
        const double measured_theta_east  = theta_x_W_Gazebo;
        const double measured_theta_north = theta_y_W_Gazebo;

        if (!is_initialized_) {
            theta_east_ = measured_theta_east;
            theta_north_ = measured_theta_north;
            is_initialized_ = true;
            return;
        }

        const double target_east = measured_theta_east + theta_rate_east_ * 0.02;
        const double target_north = measured_theta_north + theta_rate_north_ * 0.02;

        const double error_east = target_east - theta_east_;
        const double error_north = target_north - theta_north_;

        theta_east_ += 0.4 * error_east;
        theta_north_ += 0.4 * error_north;

        theta_rate_east_ += 0.1 * error_east;
        theta_rate_north_ += 0.1 * error_north;
    }

    // === [ Data Publishing ] ===
    void publish_compensated_data(rclcpp::Time stamp) {
        if (last_q_.norm() < 1e-5) return;

        // Internal world-frame state uses physical East/North naming.
        const double xw = std::tan(theta_east_);   // Gazebo World X / East
        const double yw = std::tan(theta_north_);  // Gazebo World Y / North
        const double zw = -1.0;
        Eigen::Vector3d V_world(xw, yw, zw);

        const double theta_east_deg = theta_east_ * 180.0 / M_PI;
        const double theta_north_deg = theta_north_ * 180.0 / M_PI;

        const double r_w = std::sqrt(xw * xw + yw * yw);
        const double alpha_w_deg = std::atan2(r_w, -zw) * 180.0 / M_PI;
        double beta_w_deg = 0.0;
        if (r_w > 1e-4) {
            beta_w_deg = std::atan2(yw, xw) * 180.0 / M_PI;
        }

        Eigen::Vector3d V_body = last_q_.inverse() * V_world;
        const double xb = V_body.x();
        const double yb = V_body.y();
        const double zb = V_body.z();

        const double theta_xb_deg = std::atan2(xb, -zb) * 180.0 / M_PI;
        const double theta_yb_deg = std::atan2(yb, -zb) * 180.0 / M_PI;
        const double r_b = std::sqrt(xb * xb + yb * yb);
        const double alpha_b_deg = std::atan2(r_b, -zb) * 180.0 / M_PI;
        double beta_b_deg = 0.0;
        if (r_b > 1e-4) {
            beta_b_deg = std::atan2(yb, xb) * 180.0 / M_PI;
        }

        geometry_msgs::msg::Vector3Stamped msg_b_cart;
        msg_b_cart.header.stamp = stamp;
        msg_b_cart.header.frame_id = "camera_link";
        msg_b_cart.vector.x = theta_xb_deg;
        msg_b_cart.vector.y = theta_yb_deg;
        msg_b_cart.vector.z = nominal_rope_length_;
        pub_body_cartesian_->publish(msg_b_cart);

        geometry_msgs::msg::Vector3Stamped msg_b_polar;
        msg_b_polar.header.stamp = stamp;
        msg_b_polar.header.frame_id = "camera_link";
        msg_b_polar.vector.x = alpha_b_deg;
        msg_b_polar.vector.y = beta_b_deg;
        msg_b_polar.vector.z = nominal_rope_length_;
        pub_body_polar_->publish(msg_b_polar);

        geometry_msgs::msg::Vector3Stamped msg_w_cart;
        msg_w_cart.header.stamp = stamp;
        msg_w_cart.header.frame_id = "world";

        // Keep external world/cartesian protocol unchanged:
        // vector.x = Gazebo World X = East
        // vector.y = Gazebo World Y = North
        msg_w_cart.vector.x = theta_east_deg;
        msg_w_cart.vector.y = theta_north_deg;
        msg_w_cart.vector.z = nominal_rope_length_;
        pub_world_cartesian_->publish(msg_w_cart);

        geometry_msgs::msg::Vector3Stamped msg_w_polar;
        msg_w_polar.header.stamp = stamp;
        msg_w_polar.header.frame_id = "world";
        msg_w_polar.vector.x = alpha_w_deg;
        msg_w_polar.vector.y = beta_w_deg;
        msg_w_polar.vector.z = nominal_rope_length_;
        pub_world_polar_->publish(msg_w_polar);
    }

    double cx_, cy_, fx_, fy_;
    double nominal_rope_length_;

    // World-frame physical-direction states.
    double theta_east_, theta_north_;
    double theta_rate_east_, theta_rate_north_;

    bool is_initialized_;
    rclcpp::Time last_acc_time_;
    Eigen::Quaterniond last_q_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        geometry_msgs::msg::PointStamped,
        geometry_msgs::msg::PoseStamped>;

    message_filters::Subscriber<geometry_msgs::msg::PointStamped> sub_pixel_pos_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_uav_pose_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_body_cartesian_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_body_polar_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_world_cartesian_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_world_polar_;
};

// === [ Main Function ] ===
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingAngleCVComp>());
    rclcpp::shutdown();
    return 0;
}
