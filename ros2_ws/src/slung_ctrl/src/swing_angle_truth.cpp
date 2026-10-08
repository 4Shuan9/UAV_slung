// ==============================================================================
// @File     : swing_angle_truth.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 摆角真值计算节点，基于仿真环境中物理引擎提供的绝对位姿计算标准参考真值
// 
// @Main Work: 
//   * 节点注册：创建名为 `swing_angle_calculator` 的 ROS 2 节点
//   * 数据输入：通过严格的时间同步器 (`ApproximateTime`) 订阅 Gazebo 物理引擎
//             发布的无人机 `/model/x500_mono_cam_down/pose` 与吊挂物 `/model/payload_ball/pose` 绝对位姿
//   * 核心处理：利用配置好的挂点物理偏置 (`hook_offset_body_`)，将吊挂物与相机的空间位置作向量相减得到绝对位移向量，
//             再经由无人机四元数转换阵逆变回机体坐标系；精确解算出笛卡尔投影角、极坐标偏角以及实际绳长
//   * 数据输出：将零延迟、零噪声的摆角真值发布至 `/uav/truth/swing_angle/...` 话题，
//             专用于作为 Benchamrk 与 EKF/ESO 等各种估计算法的性能表现进行对比评估
// ==============================================================================

// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

// === [ Node Definition ] ===
class SwingAngleCalculator : public rclcpp::Node {
public:
    SwingAngleCalculator() : Node("swing_angle_calculator") {
        // === [ Configuration ] ===
        hook_offset_body_ = Eigen::Vector3d(0.0, 0.0, 0.10); 

        sub_uav_.subscribe(this, "/model/x500_mono_cam_down/pose");
        sub_payload_.subscribe(this, "/model/payload_ball/pose");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_uav_, sub_payload_);
        sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.01));
        sync_->registerCallback(
            std::bind(&SwingAngleCalculator::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_body_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/truth/swing_angle/body_frame/cartesian", 10);
        pub_body_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/truth/swing_angle/body_frame/polar", 10);
        pub_world_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/truth/swing_angle/world_frame/cartesian", 10);
        pub_world_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/truth/swing_angle/world_frame/polar", 10);
            
        RCLCPP_INFO(this->get_logger(), "SwingAngleCalculator Started.");
    }

private:
    // === [ Callback & Computation ] ===
    void sync_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& uav_msg,
                       const geometry_msgs::msg::PoseStamped::ConstSharedPtr& payload_msg) {
        
        Eigen::Quaterniond q(uav_msg->pose.orientation.w, uav_msg->pose.orientation.x,
                             uav_msg->pose.orientation.y, uav_msg->pose.orientation.z);
        if (q.norm() < 1e-5) return;
        q.normalize();

        Eigen::Vector3d Pu_base(uav_msg->pose.position.x, uav_msg->pose.position.y, uav_msg->pose.position.z);
        Eigen::Vector3d Pu_cam = Pu_base + q * hook_offset_body_; 

        Eigen::Vector3d Pl(payload_msg->pose.position.x, payload_msg->pose.position.y, payload_msg->pose.position.z);
        Eigen::Vector3d r_world = Pl - Pu_cam; 
        double rope_length = r_world.norm();

        if (rope_length < 1e-5) return;

        double xw = r_world.x(), yw = r_world.y(), zw = r_world.z();
        // Gazebo World / ENU: X -> East, Y -> North
        double theta_east_deg = std::atan2(xw, -zw) * 180.0 / M_PI;
        double theta_north_deg = std::atan2(yw, -zw) * 180.0 / M_PI;
        double cos_alpha_w = std::clamp(-zw / rope_length, -1.0, 1.0);
        double alpha_w_deg = std::acos(cos_alpha_w) * 180.0 / M_PI;
        double beta_w_deg = 0.0;
        if (std::sqrt(xw * xw + yw * yw) > 1e-4) { 
            beta_w_deg = std::atan2(yw, xw) * 180.0 / M_PI;
        }

        Eigen::Vector3d r_body = q.inverse() * r_world;
        double xb = r_body.x(), yb = r_body.y(), zb = r_body.z();
        double theta_xb_deg = std::atan2(xb, -zb) * 180.0 / M_PI;
        double theta_yb_deg = std::atan2(yb, -zb) * 180.0 / M_PI;
        double cos_alpha_b = std::clamp(-zb / rope_length, -1.0, 1.0);
        double alpha_b_deg = std::acos(cos_alpha_b) * 180.0 / M_PI;
        double beta_b_deg = 0.0;
        if (std::sqrt(xb * xb + yb * yb) > 1e-4) { 
            beta_b_deg = std::atan2(yb, xb) * 180.0 / M_PI;
        }

        // === [ Data Publishing ] ===
        auto stamp = uav_msg->header.stamp;

        geometry_msgs::msg::Vector3Stamped msg_w_cart;
        msg_w_cart.header.stamp = stamp;
        msg_w_cart.header.frame_id = "world"; 
        msg_w_cart.vector.x = theta_east_deg;   // Gazebo World X / East
        msg_w_cart.vector.y = theta_north_deg;  // Gazebo World Y / North
        msg_w_cart.vector.z = rope_length;
        pub_world_cartesian_->publish(msg_w_cart);

        geometry_msgs::msg::Vector3Stamped msg_w_polar;
        msg_w_polar.header.stamp = stamp;
        msg_w_polar.header.frame_id = "world";
        msg_w_polar.vector.x = alpha_w_deg;
        msg_w_polar.vector.y = beta_w_deg;
        msg_w_polar.vector.z = rope_length;
        pub_world_polar_->publish(msg_w_polar);

        geometry_msgs::msg::Vector3Stamped msg_b_cart;
        msg_b_cart.header.stamp = stamp;
        msg_b_cart.header.frame_id = "uav_base_link"; 
        msg_b_cart.vector.x = theta_xb_deg;
        msg_b_cart.vector.y = theta_yb_deg;
        msg_b_cart.vector.z = rope_length;
        pub_body_cartesian_->publish(msg_b_cart);

        geometry_msgs::msg::Vector3Stamped msg_b_polar;
        msg_b_polar.header.stamp = stamp;
        msg_b_polar.header.frame_id = "uav_base_link";
        msg_b_polar.vector.x = alpha_b_deg;
        msg_b_polar.vector.y = beta_b_deg;
        msg_b_polar.vector.z = rope_length;
        pub_body_polar_->publish(msg_b_polar);

        // === [ Terminal Logging ] ===
        // auto& clk = *this->get_clock();
        // RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 500, 
        //     "\n"
        //     "  [真值-世界系] East: %+7.2f° | North: %+7.2f° | α: %+7.2f° | 绳长: %.3fm\n"
        //     "  [真值-机体系] X: %+7.2f° | Y: %+7.2f° | α: %+7.2f°",
        //     theta_east_deg, theta_north_deg, alpha_w_deg, rope_length,
        //     theta_xb_deg, theta_yb_deg, alpha_b_deg
        // );
    }

    Eigen::Vector3d hook_offset_body_; 

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_uav_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_payload_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_body_cartesian_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_body_polar_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_world_cartesian_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_world_polar_;
};

// === [ Main Function ] ===
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingAngleCalculator>());
    rclcpp::shutdown();
    return 0;
}