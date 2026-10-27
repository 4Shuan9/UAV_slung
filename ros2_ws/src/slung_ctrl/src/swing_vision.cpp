// ==============================================================================
// @File  : swing_vision.cpp
// @Brief : Pixel geometry + capture-time PX4 estimated attitude -> ENU swing angle.
//          Camera extrinsics and angle interfaces are unchanged.
// ==============================================================================

// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "slung_ctrl/attitude_history.hpp"
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
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

        // Offset maps PX4 sample time into the image/ROS clock. SITL uses the
        // same simulation clock, so its offset is zero. Never infer it from
        // message arrival time: that would fold transport delay into alignment.
        const double history_sec = declare_parameter("attitude_history_sec", 2.0);
        const double offset_sec = declare_parameter("attitude_time_offset", 0.0);
        const double max_clock_sec = static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e9;
        if (!std::isfinite(history_sec) || history_sec <= 0 || history_sec >= max_clock_sec ||
            !std::isfinite(offset_sec) || std::abs(offset_sec) >= max_clock_sec)
            throw std::invalid_argument("Invalid attitude clock/history configuration");
        history_ = std::make_unique<slung_ctrl::AttitudeHistory>(
            static_cast<int64_t>(history_sec * 1e9));
        attitude_offset_ns_ = static_cast<int64_t>(offset_sec * 1e9);
        sub_pixel_pos_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/uav/vision/payload_pixel", 50,
            std::bind(&SwingAngleCV::pixel_callback, this, std::placeholders::_1));
        sub_attitude_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS().keep_last(200),
            std::bind(&SwingAngleCV::attitude_callback, this, std::placeholders::_1));

        pub_body_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/angle_body", 10);
        pub_body_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/polar_body", 10);
        pub_world_cartesian_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/angle", 10);
        pub_world_polar_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/swing/vision/polar", 10);
            
        RCLCPP_INFO(get_logger(),
            "Vision: PX4 estimated attitude, capture-time SLERP, FRD/NED -> FLU/ENU; "
            "history=%.2fs, clock offset=%.6fs (no truth fallback)", history_sec, offset_sec);
    }

private:
    // === [ Capture-time attitude matching ] ===
    void check_clock() {
        const int64_t t = now().nanoseconds();
        if (last_clock_ns_ >= 0 && t < last_clock_ns_) {
            history_->clear(); pending_.clear(); last_pixel_ns_ = -1;
            RCLCPP_WARN(get_logger(), "ROS clock restarted; cleared visual attitude history");
        }
        last_clock_ns_ = t;
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {
        check_clock();
        if (msg->timestamp_sample == 0 ||
            msg->timestamp_sample > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() / 1000)) return;
        const int64_t px4_ns = static_cast<int64_t>(msg->timestamp_sample) * 1000;
        if ((attitude_offset_ns_ > 0 && px4_ns > std::numeric_limits<int64_t>::max() - attitude_offset_ns_) ||
            (attitude_offset_ns_ < 0 && px4_ns < -attitude_offset_ns_)) return;
        Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        if (!q.coeffs().allFinite() || q.norm() < 1e-6) return;
        if (history_->add(px4_ns + attitude_offset_ns_, slung_ctrl::px4_to_enu_flu(q),
                          msg->quat_reset_counter)) process_pending();
    }

    void pixel_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        check_clock();
        if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u ||
            !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) return;
        const int64_t capture = rclcpp::Time(msg->header.stamp).nanoseconds();
        if (capture <= 0 || capture <= last_pixel_ns_) return;
        last_pixel_ns_ = capture;
        pending_.push_back(msg);
        // Bounded queue while waiting for the upper attitude bracket.
        if (pending_.size() > 256) {
            pending_.pop_front();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Pixel queue full: check PX4 attitude availability and clock alignment");
        }
        process_pending();
    }

    void process_pending() {
        while (!pending_.empty()) {
            const auto pixel = pending_.front();
            Eigen::Quaterniond q;
            const auto match = history_->at(rclcpp::Time(pixel->header.stamp).nanoseconds(), q);
            if (match == slung_ctrl::AttitudeHistory::Match::wait) return;
            pending_.pop_front();
            if (match == slung_ctrl::AttitudeHistory::Match::too_old) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "No attitude bracket for image capture time (startup/reset/expired history); frame skipped");
                continue;
            }
            publish_angles(pixel, q);
        }
    }

    // Codex-generated geometry sync: sphere radius matches payload_ball/model.sdf.
    // Camera geometry is FLU. The matched quaternion maps FLU into ENU.
    void publish_angles(const geometry_msgs::msg::PointStamped::ConstSharedPtr &pixel_msg,
                        const Eigen::Quaterniond &q) {
        Eigen::Vector3d V_body;
        if (!slung_ctrl::payload_sphere_vector_body(pixel_msg->point.x, pixel_msg->point.y,
                cx_, cy_, fx_, fy_, nominal_rope_length_, slung_ctrl::payload_radius, V_body)) return;
        const double xb = V_body.x(), yb = V_body.y(), zb = V_body.z();

        Eigen::Vector3d V_world = q * V_body;
        double xw = V_world.x();
        double yw = V_world.y();
        double zw = V_world.z();

        double theta_xb_rad = std::atan2(xb, -zb);
        double theta_yb_rad = std::atan2(yb, -zb);
        double r_b = std::sqrt(xb * xb + yb * yb);
        double alpha_b_rad = std::atan2(r_b, -zb);
        double beta_b_rad = 0.0;
        if (r_b > 1e-4) { 
            beta_b_rad = std::atan2(yb, xb);
        }

        // Estimated world / ENU: X -> East, Y -> North
        double theta_east_rad = std::atan2(xw, -zw);
        double theta_north_rad = std::atan2(yw, -zw);
        double r_w = std::sqrt(xw * xw + yw * yw);
        double alpha_w_rad = std::atan2(r_w, -zw);
        double beta_w_rad = 0.0;
        if (r_w > 1e-4) { 
            beta_w_rad = std::atan2(yw, xw);
        }

        // === [ Data Publishing ] ===
        auto stamp = pixel_msg->header.stamp; 

        geometry_msgs::msg::Vector3Stamped msg_b_cart;
        msg_b_cart.header.stamp = stamp;
        msg_b_cart.header.frame_id = "uav_base_link"; 
        msg_b_cart.vector.x = theta_xb_rad;
        msg_b_cart.vector.y = theta_yb_rad;
        msg_b_cart.vector.z = nominal_rope_length_;
        pub_body_cartesian_->publish(msg_b_cart);

        geometry_msgs::msg::Vector3Stamped msg_b_polar;
        msg_b_polar.header.stamp = stamp;
        msg_b_polar.header.frame_id = "uav_base_link";
        msg_b_polar.vector.x = alpha_b_rad;
        msg_b_polar.vector.y = beta_b_rad;
        msg_b_polar.vector.z = nominal_rope_length_;
        pub_body_polar_->publish(msg_b_polar);

        geometry_msgs::msg::Vector3Stamped msg_w_cart;
        msg_w_cart.header.stamp = stamp;
        msg_w_cart.header.frame_id = "world"; 
        msg_w_cart.vector.x = theta_east_rad;   // ENU East
        msg_w_cart.vector.y = theta_north_rad;  // ENU North
        msg_w_cart.vector.z = nominal_rope_length_;
        pub_world_cartesian_->publish(msg_w_cart);

        geometry_msgs::msg::Vector3Stamped msg_w_polar;
        msg_w_polar.header.stamp = stamp;
        msg_w_polar.header.frame_id = "world";
        msg_w_polar.vector.x = alpha_w_rad;
        msg_w_polar.vector.y = beta_w_rad;
        msg_w_polar.vector.z = nominal_rope_length_;
        pub_world_polar_->publish(msg_w_polar);

    }

    double cx_, cy_, fx_, fy_;
    double nominal_rope_length_;

    int64_t attitude_offset_ns_ = 0, last_clock_ns_ = -1, last_pixel_ns_ = -1;
    std::unique_ptr<slung_ctrl::AttitudeHistory> history_;
    std::deque<geometry_msgs::msg::PointStamped::ConstSharedPtr> pending_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_pixel_pos_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_attitude_;

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