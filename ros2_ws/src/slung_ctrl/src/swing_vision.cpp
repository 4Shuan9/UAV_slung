// ==============================================================================
// @File     : swing_vision.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 视觉摆角解算节点，通过相机像素坐标直接解算三维空间下的原始观测摆角
// 
// @Main Work: 
//   * 节点注册：创建名为 `swing_vision` 的 ROS 2 节点
//   * 数据输入：订阅图像处理输出的像素坐标 `/uav/vision/payload_pixel` 
//             以及无人机在世界系下的位姿 `/model/x500_mono_cam_down/pose`，使用时间同步器 (`ApproximateTime`) 近似配对数据
//   * 核心处理：基于相机内参矩阵将 2D 像素坐标反投影为相机坐标系下的三维方向向量，
//             利用相机外参与定长约束恢复挂点到吊载的位移，再转换至世界系计算投影角与极坐标角
//   * 数据输出：将带有时间戳的解算摆角发布至 `/uav/swing/vision/...` 的机体系与世界系话题，作为后续高级估计器的原始观测输入
// ==============================================================================

// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "slung_ctrl/swing_geometry.hpp"
#include <cmath>
#include <algorithm>

// === [ Node Definition ] ===
class SwingAngleCV : public rclcpp::Node {
public:
    SwingAngleCV() : Node("swing_vision") {
        // === [ Configuration ] ===
        const double width = 640.0;
        const double height = 480.0;
        cx_ = width / 2.0;   
        cy_ = height / 2.0;  
        const double hfov = 1.74; 
        fx_ = cx_ / std::tan(hfov / 2.0); 
        fy_ = fx_; 
        nominal_rope_length_ = slung_ctrl::nominal_rope_length;

        sub_pixel_pos_.subscribe(this, "/uav/vision/payload_pixel");
        sub_uav_pose_.subscribe(this, "/model/x500_mono_cam_down/pose");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_pixel_pos_, sub_uav_pose_);
        sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.02)); 
        sync_->registerCallback(
            std::bind(&SwingAngleCV::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        pub_body_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/angle_body", 10);
        pub_body_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/polar_body", 10);
        pub_world_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/angle", 10);
        pub_world_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/polar", 10);
            
        RCLCPP_INFO(this->get_logger(), "SwingAngleCV Started.");
    }

private:
    // === [ Callback & Computation ] ===
    void sync_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& pixel_msg,
                       const geometry_msgs::msg::PoseStamped::ConstSharedPtr& uav_msg) {
        
        Eigen::Vector3d V_body;
        if (!slung_ctrl::payload_vector_body(pixel_msg->point.x, pixel_msg->point.y,
                cx_, cy_, fx_, fy_, nominal_rope_length_, V_body)) return;
        const double xb = V_body.x(), yb = V_body.y(), zb = V_body.z();

        Eigen::Quaterniond q(uav_msg->pose.orientation.w, uav_msg->pose.orientation.x,
                             uav_msg->pose.orientation.y, uav_msg->pose.orientation.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-5) return;
        q.normalize();

        Eigen::Vector3d V_world = q * V_body;
        double xw = V_world.x();
        double yw = V_world.y();
        double zw = V_world.z();

        double theta_xb_deg = std::atan2(xb, -zb) * 180.0 / M_PI;
        double theta_yb_deg = std::atan2(yb, -zb) * 180.0 / M_PI;
        double r_b = std::sqrt(xb * xb + yb * yb);
        double alpha_b_deg = std::atan2(r_b, -zb) * 180.0 / M_PI;
        double beta_b_deg = 0.0;
        if (r_b > 1e-4) { 
            beta_b_deg = std::atan2(yb, xb) * 180.0 / M_PI;
        }

        // Gazebo World / ENU: X -> East, Y -> North
        double theta_east_deg = std::atan2(xw, -zw) * 180.0 / M_PI;
        double theta_north_deg = std::atan2(yw, -zw) * 180.0 / M_PI;
        double r_w = std::sqrt(xw * xw + yw * yw);
        double alpha_w_deg = std::atan2(r_w, -zw) * 180.0 / M_PI;
        double beta_w_deg = 0.0;
        if (r_w > 1e-4) { 
            beta_w_deg = std::atan2(yw, xw) * 180.0 / M_PI;
        }

        // === [ Data Publishing ] ===
        auto stamp = pixel_msg->header.stamp; 

        geometry_msgs::msg::Vector3Stamped msg_b_cart;
        msg_b_cart.header.stamp = stamp;
        msg_b_cart.header.frame_id = "uav_base_link"; 
        msg_b_cart.vector.x = theta_xb_deg;
        msg_b_cart.vector.y = theta_yb_deg;
        msg_b_cart.vector.z = nominal_rope_length_;
        pub_body_cartesian_->publish(msg_b_cart);

        geometry_msgs::msg::Vector3Stamped msg_b_polar;
        msg_b_polar.header.stamp = stamp;
        msg_b_polar.header.frame_id = "uav_base_link";
        msg_b_polar.vector.x = alpha_b_deg;
        msg_b_polar.vector.y = beta_b_deg;
        msg_b_polar.vector.z = nominal_rope_length_;
        pub_body_polar_->publish(msg_b_polar);

        geometry_msgs::msg::Vector3Stamped msg_w_cart;
        msg_w_cart.header.stamp = stamp;
        msg_w_cart.header.frame_id = "world"; 
        msg_w_cart.vector.x = theta_east_deg;   // Gazebo World X / East
        msg_w_cart.vector.y = theta_north_deg;  // Gazebo World Y / North
        msg_w_cart.vector.z = nominal_rope_length_;
        pub_world_cartesian_->publish(msg_w_cart);

        geometry_msgs::msg::Vector3Stamped msg_w_polar;
        msg_w_polar.header.stamp = stamp;
        msg_w_polar.header.frame_id = "world";
        msg_w_polar.vector.x = alpha_w_deg;
        msg_w_polar.vector.y = beta_w_deg;
        msg_w_polar.vector.z = nominal_rope_length_;
        pub_world_polar_->publish(msg_w_polar);

        // === [ Terminal Logging ] ===
        // auto& clk = *this->get_clock();
        // RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 500, 
        //     "\n"
        //     "  [视觉-世界系] East: %+7.2f° | North: %+7.2f° | α: %+7.2f°\n"
        //     "  [视觉-机体系] X: %+7.2f° | Y: %+7.2f° | α: %+7.2f°",
        //     theta_east_deg, theta_north_deg, alpha_w_deg,
        //     theta_xb_deg, theta_yb_deg, alpha_b_deg
        // );
    }

    double cx_, cy_, fx_, fy_;
    double nominal_rope_length_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PoseStamped>;
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> sub_pixel_pos_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> sub_uav_pose_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_body_cartesian_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_body_polar_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_world_cartesian_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_world_polar_;
};

// === [ Main Function ] ===
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingAngleCV>());
    rclcpp::shutdown();
    return 0;
}