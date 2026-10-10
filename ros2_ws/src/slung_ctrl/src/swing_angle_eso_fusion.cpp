// === [ Includes ] ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "slung_ctrl/msg/swing_adrc_state.hpp"
#include <cmath>
#include <mutex>
#include <algorithm>
#include <utility>

using namespace std::chrono_literals;

// === [ Node Definition ] ===
class SwingAngleESOFusion : public rclcpp::Node {
public:
    SwingAngleESOFusion() : Node("swing_angle_eso_fusion") {
        this->declare_parameter("g", 9.81);
        this->declare_parameter("l", 1.0);
        this->declare_parameter("omega_o", 8.0);
        this->declare_parameter("tau", 0.030);
        this->declare_parameter("dt", 0.005);
        this->declare_parameter("filter_alpha", 0.5);

        // Delay compensation: fixed tau is the safe default; dynamic age is optional.
        this->declare_parameter("use_dynamic_delay", false);
        this->declare_parameter("dynamic_delay_max", 0.100);

        this->get_parameter("g", gravity_);
        this->get_parameter("l", cable_length_);
        this->get_parameter("omega_o", bandwidth_);
        this->get_parameter("tau", delay_compensation_time_);
        this->get_parameter("dt", time_step_);
        this->get_parameter("filter_alpha", filter_alpha_);
        this->get_parameter("use_dynamic_delay", use_dynamic_delay_);
        this->get_parameter("dynamic_delay_max", dynamic_delay_max_);

        update_gains();

        param_subscriber_ = this->add_on_set_parameters_callback(
            std::bind(&SwingAngleESOFusion::parameters_callback, this, std::placeholders::_1));

        first_filter_run_ = true;
        first_imu_run_ = true;
        has_vision_ = false;

        // Use the aligned visual message as the only visual input to this
        // estimator, so angle and timestamp always belong to the same frame.
        vision_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/cv/swing_angle_aligned/world_frame/cartesian", 10,
            std::bind(&SwingAngleESOFusion::vision_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(),
            std::bind(&SwingAngleESOFusion::imu_callback, this, std::placeholders::_1));

        fusion_angle_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/eso_fusion/swing_angle/world_frame/cartesian", 10);
        fusion_rate_publisher_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/eso_fusion/swing_rate/world_frame/cartesian", 10);
        fusion_dist_publisher_  = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/uav/eso_fusion/disturbance/cartesian", 10);

        // Synchronized state snapshot for ADRC. All six control quantities are
        // generated from the same LESO update and published in one message.
        adrc_state_publisher_ = this->create_publisher<slung_ctrl::msg::SwingADRCState>(
            "/uav/eso_fusion/adrc_state", 10);
    }

private:
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        std::lock_guard<std::mutex> lock(data_mutex_);
        for (const auto &param : parameters) {
            if (param.get_name() == "g") {
                if (param.as_double() <= 0.0) { result.successful = false; result.reason = "Invalid g"; return result; }
                gravity_ = param.as_double();
            } else if (param.get_name() == "l") {
                if (param.as_double() <= 0.01) { result.successful = false; result.reason = "Invalid l"; return result; }
                cable_length_ = param.as_double();
            } else if (param.get_name() == "omega_o") {
                if (param.as_double() <= 0.0) { result.successful = false; result.reason = "Invalid omega_o"; return result; }
                bandwidth_ = param.as_double();
            } else if (param.get_name() == "tau") {
                if (param.as_double() < 0.0) { result.successful = false; result.reason = "Invalid tau"; return result; }
                delay_compensation_time_ = param.as_double();
            } else if (param.get_name() == "filter_alpha") {
                if (param.as_double() < 0.0 || param.as_double() > 1.0) { result.successful = false; result.reason = "Invalid filter_alpha"; return result; }
                filter_alpha_ = param.as_double();
            } else if (param.get_name() == "dt") {
                if (param.as_double() <= 0.0) { result.successful = false; result.reason = "Invalid dt"; return result; }
                time_step_ = param.as_double();
            } else if (param.get_name() == "use_dynamic_delay") {
                use_dynamic_delay_ = param.as_bool();
                RCLCPP_INFO(
                    this->get_logger(),
                    "Delay compensation mode: %s",
                    use_dynamic_delay_ ? "DYNAMIC" : "FIXED");
            } else if (param.get_name() == "dynamic_delay_max") {
                if (param.as_double() <= 0.0 || param.as_double() > 1.0) {
                    result.successful = false;
                    result.reason = "Invalid dynamic_delay_max";
                    return result;
                }
                dynamic_delay_max_ = param.as_double();
            }
        }
        
        update_gains();
        return result;
    }

    void update_gains() {
        input_gain_ = -1.0 / cable_length_;
        observer_gain_1_ = 3.0 * bandwidth_;
        observer_gain_2_ = 3.0 * std::pow(bandwidth_, 2) - (gravity_ / cable_length_);
        observer_gain_3_ = std::pow(bandwidth_, 3);
    }

    void vision_callback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Aligned message values are identical to the raw visual measurement.
        // Gazebo World / ENU: X -> East, Y -> North.
        measured_angle_east_  = msg->vector.x * M_PI / 180.0;
        measured_angle_north_ = msg->vector.y * M_PI / 180.0;

        // Aligned header.stamp is already in ROS/system time, i.e. the same
        // time domain used by this Fusion node.
        const auto now = this->get_clock()->now();
        const rclcpp::Time measurement_time(msg->header.stamp);
        const double measured_delay_s = (now - measurement_time).seconds();

        if (measured_delay_s >= 0.0 && measured_delay_s <= dynamic_delay_max_) {
            latest_measured_delay_s_ = measured_delay_s;
            has_measured_delay_ = true;
        } else {
            // Keep the visual angle, but never allow an invalid timestamp to
            // drive the predictor. Dynamic mode falls back to fixed tau.
            has_measured_delay_ = false;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Invalid aligned visual delay %.3f ms; using fixed tau.",
                measured_delay_s * 1000.0);
        }

        has_vision_ = true;
    }

    void imu_callback(const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Source variables in PX4 local frame (NED):
        // PX4 X -> North, PX4 Y -> East.
        const double accel_x_PX4 = msg->ax;
        const double accel_y_PX4 = msg->ay;

        // Keep the existing, physically correct channel pairing unchanged.
        accel_north_ = accel_x_PX4;
        accel_east_  = accel_y_PX4;

        if (!has_vision_) {
            return;
        }

        auto now = this->get_clock()->now();
        if (first_imu_run_) {
            last_imu_time_ = now;
            first_imu_run_ = false;
            return;
        }

        // 核心修复：计算真实的事件触发时间差 (动态 dt)
        double current_dt = (now - last_imu_time_).seconds();
        last_imu_time_ = now;

        // 安全防线：防止异常掉帧或网络卡顿导致积分发散
        if (current_dt <= 0.0 || current_dt > 0.1) {
            current_dt = time_step_; 
        }

        double error_east = measured_angle_east_ - eso_state_east_.angle;
        double error_north = measured_angle_north_ - eso_state_north_.angle;

        // 使用真实的 current_dt 进行欧拉积分
        auto update_eso = [&](LesoState &state, double accel_input, double error) {
            state.angle += (state.rate + observer_gain_1_ * error) * current_dt;
            state.rate += (-(gravity_ / cable_length_) * state.angle + input_gain_ * accel_input + state.disturbance + observer_gain_2_ * error) * current_dt;
            state.disturbance += (observer_gain_3_ * error) * current_dt;
        };
        
        update_eso(eso_state_east_, accel_east_, error_east);
        update_eso(eso_state_north_, accel_north_, error_north);

        // Prediction horizon:
        //   fixed mode   -> configured tau
        //   dynamic mode -> latest valid visual sample age measured at Fusion
        // Invalid / unavailable dynamic timing falls back to fixed tau.
        double prediction_delay_s = delay_compensation_time_;
        if (use_dynamic_delay_ && has_measured_delay_) {
            prediction_delay_s = std::clamp(
                latest_measured_delay_s_,
                0.0,
                dynamic_delay_max_);
        }

        auto predict_state = [&](const LesoState &state, double accel_input) {
            double predicted_accel =
                -(gravity_ / cable_length_) * state.angle
                + input_gain_ * accel_input
                + state.disturbance;

            return std::make_pair(
                state.angle
                    + state.rate * prediction_delay_s
                    + 0.5 * predicted_accel
                        * prediction_delay_s * prediction_delay_s,
                state.rate + predicted_accel * prediction_delay_s
            );
        };

        auto prediction_east = predict_state(eso_state_east_, accel_east_);
        auto prediction_north = predict_state(eso_state_north_, accel_north_);

        if (first_filter_run_) {
            filtered_angle_east_ = prediction_east.first;
            filtered_angle_north_ = prediction_north.first;
            filtered_rate_east_  = prediction_east.second;
            filtered_rate_north_  = prediction_north.second;
            filtered_dist_east_  = eso_state_east_.disturbance;
            filtered_dist_north_  = eso_state_north_.disturbance;
            first_filter_run_ = false;
        } else {
            filtered_angle_east_ = filter_alpha_ * prediction_east.first  + (1.0 - filter_alpha_) * filtered_angle_east_;
            filtered_angle_north_ = filter_alpha_ * prediction_north.first  + (1.0 - filter_alpha_) * filtered_angle_north_;
            filtered_rate_east_  = filter_alpha_ * prediction_east.second + (1.0 - filter_alpha_) * filtered_rate_east_;
            filtered_rate_north_  = filter_alpha_ * prediction_north.second + (1.0 - filter_alpha_) * filtered_rate_north_;
            filtered_dist_east_  = filter_alpha_ * eso_state_east_.disturbance   + (1.0 - filter_alpha_) * filtered_dist_east_;
            filtered_dist_north_  = filter_alpha_ * eso_state_north_.disturbance   + (1.0 - filter_alpha_) * filtered_dist_north_;
        }

        // ------------------------------------------------------------------
        // Publish one coherent ADRC state snapshot.
        // IMPORTANT: during this synchronization stage the units intentionally
        // match the existing public topics so the old and new interfaces can be
        // compared sample-for-sample without changing controller numerics.
        // ------------------------------------------------------------------
        slung_ctrl::msg::SwingADRCState msg_adrc_state;
        msg_adrc_state.header.stamp = now;
        msg_adrc_state.header.frame_id = "world";
        msg_adrc_state.theta_east_deg = filtered_angle_east_ * 180.0 / M_PI;
        msg_adrc_state.theta_north_deg = filtered_angle_north_ * 180.0 / M_PI;
        msg_adrc_state.rate_east_deg_s = filtered_rate_east_ * 180.0 / M_PI;
        msg_adrc_state.rate_north_deg_s = filtered_rate_north_ * 180.0 / M_PI;
        msg_adrc_state.disturbance_east_rad_s2 = filtered_dist_east_;
        msg_adrc_state.disturbance_north_rad_s2 = filtered_dist_north_;
        adrc_state_publisher_->publish(msg_adrc_state);

        // Keep the three public estimator topics for PlotJuggler and offline analysis.
        geometry_msgs::msg::Vector3Stamped msg_angle, msg_rate, msg_dist;
        msg_angle.header.stamp = msg_rate.header.stamp = msg_dist.header.stamp = now;
        msg_angle.header.frame_id = "world";

        // External message protocol remains unchanged:
        // vector.x = Gazebo World X = East
        // vector.y = Gazebo World Y = North
        msg_angle.vector.x = filtered_angle_east_ * 180.0 / M_PI; 
        msg_angle.vector.y = filtered_angle_north_ * 180.0 / M_PI;
        fusion_angle_publisher_->publish(msg_angle);

        msg_rate.vector.x = filtered_rate_east_ * 180.0 / M_PI; 
        msg_rate.vector.y = filtered_rate_north_ * 180.0 / M_PI;
        fusion_rate_publisher_->publish(msg_rate);

        msg_dist.vector.x = filtered_dist_east_; 
        msg_dist.vector.y = filtered_dist_north_;
        fusion_dist_publisher_->publish(msg_dist);
    }

    struct LesoState { double angle = 0.0, rate = 0.0, disturbance = 0.0; };
    LesoState eso_state_east_, eso_state_north_; 

    double accel_east_ = 0.0, accel_north_ = 0.0;     
    double measured_angle_east_ = 0.0, measured_angle_north_ = 0.0;     
    bool has_vision_;

    // Delay-compensation timing state.
    double latest_measured_delay_s_ = 0.0;
    bool has_measured_delay_ = false;
    bool use_dynamic_delay_ = false;
    double dynamic_delay_max_ = 0.100;

    rclcpp::Time last_imu_time_;
    bool first_imu_run_;

    double filter_alpha_;
    bool first_filter_run_;
    double filtered_angle_east_ = 0.0, filtered_angle_north_ = 0.0;
    double filtered_rate_east_  = 0.0, filtered_rate_north_  = 0.0;
    double filtered_dist_east_  = 0.0, filtered_dist_north_  = 0.0;

    std::mutex data_mutex_;

    double gravity_, cable_length_, input_gain_, bandwidth_, delay_compensation_time_, time_step_;
    double observer_gain_1_, observer_gain_2_, observer_gain_3_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr fusion_angle_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr fusion_rate_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr fusion_dist_publisher_;
    rclcpp::Publisher<slung_ctrl::msg::SwingADRCState>::SharedPtr adrc_state_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;
};

// === [ Main Function ] ===
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwingAngleESOFusion>());
    rclcpp::shutdown();
    return 0;
}
