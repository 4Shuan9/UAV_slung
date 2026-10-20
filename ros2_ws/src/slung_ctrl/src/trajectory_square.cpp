// ==============================================================================
// @File     : trajectory_square.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 菱形/方形轨迹生成节点，用于测试无人机在断点转折和直线巡航下的瞬态防摆响应
// 
// @Main Work: 
//   * 节点注册：创建名为 `trajectory_square` 的 ROS 2 节点
//   * 数据输入：订阅无人机里程计 `/fmu/out/vehicle_odometry`，用于初始化时锁定起点位置及悬停基准高度
//   * 核心处理：根据参数边长 (`square_side_length`) 计算4个相对角点坐标。在 50Hz 的控制循环中，以配置的巡航速度 (`cruise_speed`) 沿直线平滑插值移动目标点；
//             到达当前角点后自动切换下一航点，走完全程后节点自动安全退出
//   * 数据输出：将插值平滑后的期望坐标发布至 `/uav/reference/position` (消息类型: Point)
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class SlungAntiswingSquare : public rclcpp::Node {
public:
    SlungAntiswingSquare() : Node("trajectory_square") {
        this->declare_parameter("square_side_length", 10.0); 
        this->declare_parameter("cruise_speed", 3.0);       

        pub_target = this->create_publisher<geometry_msgs::msg::Point>("/uav/reference/position", 10);
        
        sub_odom = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&SlungAntiswingSquare::odom_cb, this, std::placeholders::_1));

        timer = this->create_wall_timer(20ms, std::bind(&SlungAntiswingSquare::timer_cb, this));
        
        RCLCPP_INFO(this->get_logger(), "Rhombus Generator Started.");
    }

private:
    float target_x = 0.0, target_y = 0.0, target_z = 0.0; 
    bool init_done = false;

    std::vector<std::pair<float, float>> corners; 
    int current_corner_idx = 0;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom;
    rclcpp::TimerBase::SharedPtr timer;

    // 获取起飞悬停点与基准高度
    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        if (!init_done) {
            target_x = msg->position[0];
            target_y = msg->position[1];
            target_z = msg->position[2]; // 锁定起飞悬停高度

            float L = this->get_parameter("square_side_length").as_double();
            float D = L * 0.70710678f; 

            // 生成菱形四角点
            corners.push_back({target_x + D,   target_y + D});
            corners.push_back({target_x + 2*D, target_y});    
            corners.push_back({target_x + D,   target_y - D});
            corners.push_back({target_x,       target_y});    

            init_done = true;
            RCLCPP_INFO(this->get_logger(), "Init Pos Locked. Generating Rhombus (Side: %.1fm)", L);
        }
    }

    // 50Hz 轨迹发布
    void timer_cb() {
        if (!init_done) return;

        float dest_x = corners[current_corner_idx].first;
        float dest_y = corners[current_corner_idx].second;
        
        float dir_x = dest_x - target_x;
        float dir_y = dest_y - target_y;
        float dist = std::sqrt(dir_x * dir_x + dir_y * dir_y);

        float step = this->get_parameter("cruise_speed").as_double() * 0.02; 

        if (dist < step) {
            target_x = dest_x;
            target_y = dest_y;
            current_corner_idx++;
            RCLCPP_INFO(this->get_logger(), "Reached Corner %d/4", current_corner_idx);

            if (current_corner_idx >= 4) {
                RCLCPP_INFO(this->get_logger(), "Rhombus Finished! Exiting Node.");
                rclcpp::shutdown(); // 走完自动退出
                return;
            }
        } else {
            target_x += (dir_x / dist) * step;
            target_y += (dir_y / dist) * step;
        }

        // 下发目标位置
        geometry_msgs::msg::Point pt;
        pt.x = target_x;
        pt.y = target_y;
        pt.z = target_z; // 维持锁定高度
        pub_target->publish(pt);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlungAntiswingSquare>());
    return 0;
}