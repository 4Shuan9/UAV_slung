// ==============================================================================
// @File     : trajectory_eight.cpp
// @Author   : 4Shuan9
// @Date     : 2026-09-08
// @Brief    : 8字轨迹生成节点，新增轨迹切线航向(Yaw)角随动解算
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>

using namespace std::chrono_literals;

class SlungAntiswingEight : public rclcpp::Node {
public:
    SlungAntiswingEight() : Node("trajectory_eight") {
        this->declare_parameter("eight_scale", 8.0); 
        this->declare_parameter("omega", 0.150);

        pub_target = this->create_publisher<geometry_msgs::msg::Point>("/uav/reference/position", 10);
        pub_yaw = this->create_publisher<std_msgs::msg::Float32>("/uav/reference/yaw", 10); // 新增Yaw发布器
        
        sub_odom = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&SlungAntiswingEight::odom_cb, this, std::placeholders::_1));

        timer = this->create_wall_timer(20ms, std::bind(&SlungAntiswingEight::timer_cb, this));
        
        RCLCPP_INFO(this->get_logger(), "Figure-8 trajectory node started (with Tangent Yaw).");
    }

private:
    float center_x = 0.0, center_y = 0.0, center_z = 0.0; 
    bool init_done = false;
    double t = 0.0; 
    int current_loop = 0;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw; // 新增Yaw发布器
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom;
    rclcpp::TimerBase::SharedPtr timer;

    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        if (!init_done) {
            center_x = msg->position[0];
            center_y = msg->position[1];
            center_z = msg->position[2]; 
            init_done = true;
            RCLCPP_INFO(this->get_logger(), "Center and Z-altitude locked. Starting trajectory.");
        }
    }

    void timer_cb() {
        if (!init_done) return;

        float scale = this->get_parameter("eight_scale").as_double();
        float omega = this->get_parameter("omega").as_double();

        t += 0.02; 

        int calc_loop = static_cast<int>((omega * t) / (2.0 * M_PI)) + 1;
        if (calc_loop > current_loop) {
            current_loop = calc_loop;
            RCLCPP_INFO(this->get_logger(), ">>> Executing Figure-8 Loop: %d <<<", current_loop);
        }

        // 1. 计算坐标点
        geometry_msgs::msg::Point pt;
        pt.x = center_x + scale * std::sin(omega * t);
        pt.y = center_y + scale * std::sin(omega * t) * std::cos(omega * t);
        pt.z = center_z; 
        pub_target->publish(pt);

        // 2. 核心修改：对运动方程求导，解算 X/Y 轴瞬时速度，推导切线航向角
        float vx = scale * omega * std::cos(omega * t);
        float vy = scale * omega * std::cos(2.0 * omega * t); 

        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = std::atan2(vy, vx); 
        pub_yaw->publish(yaw_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlungAntiswingEight>());
    rclcpp::shutdown();
    return 0;
}