// ==============================================================================
// @File     : slung_antiswing_PD.cpp
// @Author   : 4Shuan9
// @Date     : 2026-09-08
// @Brief    : 核心飞行控制与防摆节点，结合位置追踪与吊挂防摆算法，生成并下发底层指令
// 
// @Main Work: 
//      1. 状态机管理：实现 INIT(初始化与安全解锁)、TAKEOFF(平滑起飞)、CONTROL(防摆追踪) 三阶段飞行状态机，并支持空中无缝接管。
//      2. 轨迹与姿态解算：融合PX4里程计与ESO(扩展状态观测器)输出的摆角/角速度数据，提取真实偏航角并锁定。
//      3. 核心防摆算法：在 XY 轴采用“位置PD + 摆角PD”双环控制，解算带幅值限幅的加速度设定点(Acceleration Setpoint)；
//         Z 轴维持位置控制，最终通过Offboard模式下发给PX4，实现精准的航点追踪与吊挂减摇。
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cmath>
#include <mutex>
#include <algorithm>

using namespace std::chrono_literals;

enum class FlightState { INIT, TAKEOFF, CONTROL };

class SlungAntiswing : public rclcpp::Node {
public:
    SlungAntiswing() : Node("slung_antiswing") {
        this->declare_parameter("takeoff_height", 5.0);   
        this->declare_parameter("takeoff_speed", 0.5);    
        this->declare_parameter("kp_pos", 1.2);       
        this->declare_parameter("kd_pos", 0.8);       
        this->declare_parameter("kp_swing", 1.5);     
        this->declare_parameter("kd_swing", 0.5);     
        this->declare_parameter("max_comp_acc", 3.0); 

        pub_offboard_mode = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        pub_trajectory = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        pub_cmd = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        sub_eso_angle = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/eso_fusion/swing_angle/world_frame/cartesian", 10,
            std::bind(&SlungAntiswing::angle_cb, this, std::placeholders::_1));
        
        sub_eso_rate = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/uav/eso_fusion/swing_rate/world_frame/cartesian", 10,
            std::bind(&SlungAntiswing::rate_cb, this, std::placeholders::_1));

        sub_odom = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&SlungAntiswing::odom_cb, this, std::placeholders::_1));

        sub_target_pos = this->create_subscription<geometry_msgs::msg::Point>(
            "/uav/target_position", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                if (state_ == FlightState::CONTROL) {
                    // /uav/target_position uses PX4 local/NED horizontal semantics:
                    // msg->x -> North, msg->y -> East
                    target_north = msg->x;
                    target_east = msg->y;
                    target_z = msg->z;
                }
            });

        sub_target_yaw = this->create_subscription<std_msgs::msg::Float32>(
            "/uav/target_yaw", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (state_ == FlightState::CONTROL) target_yaw = msg->data;
            });

        timer = this->create_wall_timer(20ms, std::bind(&SlungAntiswing::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Acceleration Anti-swing Node Init. Waiting for Odom...");
    }

private:
    FlightState state_ = FlightState::INIT;
    int offboard_counter = 0;
    std::mutex data_mutex;

    double theta_east = 0, theta_north = 0, d_theta_east = 0, d_theta_north = 0;
    float current_north = 0, current_east = 0, current_z = 0; 
    float current_v_north = 0, current_v_east = 0;
    float target_north = 0, target_east = 0, target_z = 0; 
    float target_yaw = 0.0f; 
    float current_sp_z = 0; 
    
    bool odom_received = false;
    bool is_mid_air = false; 

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr pub_offboard_mode;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pub_trajectory;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr pub_cmd;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_eso_angle;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_eso_rate;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_pos;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_target_yaw;
    rclcpp::TimerBase::SharedPtr timer;

    void angle_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex);

        // ESO Fusion world-frame protocol: vector.x -> East, vector.y -> North
        theta_east = msg->vector.x;
        theta_north = msg->vector.y;
    }

    void rate_cb(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex);

        // ESO Fusion world-frame protocol: vector.x -> East rate, vector.y -> North rate
        d_theta_east = msg->vector.x;
        d_theta_north = msg->vector.y;
    }

    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        // PX4 local frame is NED: X -> North, Y -> East, Z -> Down
        current_north = msg->position[0];
        current_east = msg->position[1];
        current_z = msg->position[2];
        
        current_v_north = msg->velocity[0];
        current_v_east = msg->velocity[1];

        if (!odom_received) {
            target_north = current_north;
            target_east = current_east;
            current_sp_z = current_z;

            float w = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];
            target_yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

            if (current_z < -0.5f) {
                is_mid_air = true;
                RCLCPP_INFO(this->get_logger(), ">>> Mid-air Restart Detected! Resuming Hover at Z: %.2fm", current_z);
            } else {
                RCLCPP_INFO(this->get_logger(), "Ground Start Detected. Initial Yaw: %.4f rad.", target_yaw);
            }

            odom_received = true;
        }
    }

    void control_loop() {
        if (!odom_received) return;

        publish_offboard_control_mode();
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        sp.yaw = target_yaw; 

        if (state_ == FlightState::INIT) {
            sp.position = {target_north, target_east, current_sp_z};
            sp.velocity = {NAN, NAN, NAN};
            sp.acceleration = {NAN, NAN, NAN};
            pub_trajectory->publish(sp);
            
            if (++offboard_counter == 15) {
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); 
                publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); 
                
                target_z = is_mid_air ? current_sp_z : -(this->get_parameter("takeoff_height").as_double());
            }
            if (offboard_counter >= 20) {
                if (is_mid_air) {
                    state_ = FlightState::CONTROL;
                    RCLCPP_INFO(this->get_logger(), "Anti-Swing Resumed Seamlessly (Accel Mode).");
                } else {
                    state_ = FlightState::TAKEOFF;
                }
            }
            return;
        }

        if (state_ == FlightState::TAKEOFF) {
            double speed = this->get_parameter("takeoff_speed").as_double();
            if (current_sp_z > target_z) {
                current_sp_z -= (speed * 0.02); 
            } else {
                current_sp_z = target_z;
                if (std::abs(current_z - target_z) < 0.3) {
                    state_ = FlightState::CONTROL; 
                    RCLCPP_INFO(this->get_logger(), "Takeoff Done. Accel Anti-Swing Active.");
                }
            }
            sp.position = {target_north, target_east, current_sp_z};
            sp.velocity = {0.0f, 0.0f, (float)-speed}; 
            sp.acceleration = {NAN, NAN, NAN};
            pub_trajectory->publish(sp);
            return; 
        }

        if (state_ == FlightState::CONTROL) {
            double kp_pos = this->get_parameter("kp_pos").as_double();   
            double kd_pos = this->get_parameter("kd_pos").as_double();
            double kp_swing = this->get_parameter("kp_swing").as_double(); 
            double kd_swing = this->get_parameter("kd_swing").as_double(); 
            double a_max = this->get_parameter("max_comp_acc").as_double();

            double theta_east_rad, theta_north_rad, rate_east_rad, rate_north_rad;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                theta_east_rad = theta_east * M_PI / 180.0;
                theta_north_rad = theta_north * M_PI / 180.0;
                rate_east_rad = d_theta_east * M_PI / 180.0;
                rate_north_rad = d_theta_north * M_PI / 180.0;
            }

            // North channel: PX4 acceleration[0] <-> theta_north
            const float position_pd_north =
                kp_pos * (target_north - current_north) - kd_pos * current_v_north;
            const float swing_pd_north =
                kp_swing * std::sin(theta_north_rad)
                + kd_swing * rate_north_rad * std::cos(theta_north_rad);
            const float accel_north = position_pd_north + swing_pd_north;

            // East channel: PX4 acceleration[1] <-> theta_east
            const float position_pd_east =
                kp_pos * (target_east - current_east) - kd_pos * current_v_east;
            const float swing_pd_east =
                kp_swing * std::sin(theta_east_rad)
                + kd_swing * rate_east_rad * std::cos(theta_east_rad);
            const float accel_east = position_pd_east + swing_pd_east;

            sp.position = {NAN, NAN, target_z}; 
            sp.velocity = {NAN, NAN, NAN}; 
            sp.acceleration = {
                std::clamp(accel_north, (float)-a_max, (float)a_max),
                std::clamp(accel_east, (float)-a_max, (float)a_max),
                NAN
            }; 
            
            pub_trajectory->publish(sp);
        }
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true;
        msg.velocity = true; 
        msg.acceleration = true; 
        pub_offboard_mode->publish(msg);
    }

    void publish_vehicle_command(uint16_t cmd, float p1 = 0.0, float p2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = p1; msg.param2 = p2; msg.command = cmd;
        msg.target_system = 1; msg.target_component = 1;
        msg.source_system = 1; msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub_cmd->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlungAntiswing>());
    rclcpp::shutdown();
    return 0;
}