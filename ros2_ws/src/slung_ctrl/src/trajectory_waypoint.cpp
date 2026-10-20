// ==============================================================================
// @File     : trajectory_waypoint.cpp
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 航点交互与键盘遥控生成节点，用于手动安全地下发实时目标航点与偏航角指令
// 
// @Main Work: 
//   * 节点注册：创建名为 `trajectory_waypoint` 的 ROS 2 节点
//   * 数据输入：
//       - 状态获取：订阅 `/fmu/out/vehicle_odometry` 获取初始位姿及偏航角
//       - 外部指令：订阅 `/uav/reference/waypoint` 接收外部程序发送的航点
//       - 键盘监听：通过后台独立线程非阻塞监听终端的键盘输入 (W/A/S/D/Z/C/Q/E)
//   * 核心处理：将键盘按键转化为三轴坐标与偏航角的步进偏移量。为防止阶跃指令导致无人机失控，在 50Hz 定时器内，
//             按设定的巡航速度 (`cruise_speed`) 和航向步长 (`yaw_step`) 对目标状态进行平滑限幅插值逼近
//   * 数据输出：发布平滑处理后的期望位置至 `/uav/reference/position`，期望偏航角发布至 `/uav/reference/yaw`
// ==============================================================================

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cmath>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <atomic>

using namespace std::chrono_literals;

class SlungAntiswingWaypoint : public rclcpp::Node {
public:
    SlungAntiswingWaypoint() : Node("trajectory_waypoint") {
        this->declare_parameter("cruise_speed", 3.0);
        this->declare_parameter("kb_step_xy", 0.5);  
        this->declare_parameter("kb_step_z", 0.3);   
        this->declare_parameter("kb_step_yaw", 0.2); 

        pub_target = this->create_publisher<geometry_msgs::msg::Point>("/uav/reference/position", 10);
        pub_yaw = this->create_publisher<std_msgs::msg::Float32>("/uav/reference/yaw", 10);
        
        sub_odom = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
            std::bind(&SlungAntiswingWaypoint::odom_cb, this, std::placeholders::_1));

        sub_waypoint = this->create_subscription<geometry_msgs::msg::Point>(
            "/uav/reference/waypoint", 10,
            std::bind(&SlungAntiswingWaypoint::waypoint_cb, this, std::placeholders::_1));

        timer = this->create_wall_timer(20ms, std::bind(&SlungAntiswingWaypoint::timer_cb, this));
        
        print_instructions();
        kb_thread_ = std::thread(&SlungAntiswingWaypoint::keyboard_loop, this);
    }

    ~SlungAntiswingWaypoint() {
        if (kb_thread_.joinable()) kb_thread_.join();
    }

private:
    std::atomic<float> current_x{0}, current_y{0}, current_z{0}, current_yaw{0.0f}; 
    std::atomic<float> dest_x{0}, dest_y{0}, dest_z{0}, dest_yaw{0.0f}; 
    bool init_done = false;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_target;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_waypoint;
    rclcpp::TimerBase::SharedPtr timer;
    std::thread kb_thread_;

    void print_instructions() {
        RCLCPP_INFO(this->get_logger(), "\n"
            "==========================================\n"
            "   Real-time Keyboard Controller Active!  \n"
            "------------------------------------------\n"
            "   [W] Forward      [S] Backward          \n"
            "   [A] Left         [D] Right             \n"
            "   [Z] Ascend       [C] Descend           \n"
            "   [Q] Yaw Left     [E] Yaw Right         \n"
            "==========================================");
    }

    // 后台非阻塞键盘监听
    void keyboard_loop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 1; 
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok()) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) > 0 && init_done) {
                float step_xy = this->get_parameter("kb_step_xy").as_double();
                float step_z = this->get_parameter("kb_step_z").as_double();
                float step_yaw = this->get_parameter("kb_step_yaw").as_double();
                float yaw = current_yaw.load();

                switch (c) {
                    case 'w': case 'W': 
                        dest_x = current_x.load() + step_xy * std::cos(yaw);
                        dest_y = current_y.load() + step_xy * std::sin(yaw); break;
                    case 's': case 'S': 
                        dest_x = current_x.load() - step_xy * std::cos(yaw);
                        dest_y = current_y.load() - step_xy * std::sin(yaw); break;
                    case 'a': case 'A': 
                        dest_x = current_x.load() + step_xy * std::sin(yaw);
                        dest_y = current_y.load() - step_xy * std::cos(yaw); break;
                    case 'd': case 'D': 
                        dest_x = current_x.load() - step_xy * std::sin(yaw);
                        dest_y = current_y.load() + step_xy * std::cos(yaw); break;
                    case 'z': case 'Z': dest_z = current_z.load() - step_z; break; 
                    case 'c': case 'C': dest_z = current_z.load() + step_z; break;
                    case 'q': case 'Q': dest_yaw = current_yaw.load() - step_yaw; break;
                    case 'e': case 'E': dest_yaw = current_yaw.load() + step_yaw; break;
                }
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    // 锁定初始状态，并解算四元数防止偏航角突变
    void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        if (!init_done) {
            current_x = dest_x = msg->position[0];
            current_y = dest_y = msg->position[1];
            current_z = dest_z = msg->position[2];

            float w = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];
            float initial_yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
            current_yaw = dest_yaw = initial_yaw;

            init_done = true;
        }
    }

    void waypoint_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (init_done) {
            dest_x = msg->x;
            dest_y = msg->y;
            dest_z = msg->z;
        }
    }

    // 50Hz 平滑逼近与指令下发
    void timer_cb() {
        if (!init_done) return;

        float dir_x = dest_x.load() - current_x.load();
        float dir_y = dest_y.load() - current_y.load();
        float dir_z = dest_z.load() - current_z.load();
        float dist = std::sqrt(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);

        float step = this->get_parameter("cruise_speed").as_double() * 0.02; 

        if (dist > step) {
            current_x = current_x.load() + (dir_x / dist) * step;
            current_y = current_y.load() + (dir_y / dist) * step;
            current_z = current_z.load() + (dir_z / dist) * step;
        } else {
            current_x = dest_x.load();
            current_y = dest_y.load();
            current_z = dest_z.load();
        }

        float yaw_diff = dest_yaw.load() - current_yaw.load();
        float yaw_step = 1.0 * 0.02; 
        if (std::abs(yaw_diff) > yaw_step) {
            current_yaw = current_yaw.load() + (yaw_diff > 0 ? yaw_step : -yaw_step);
        } else {
            current_yaw = dest_yaw.load();
        }

        geometry_msgs::msg::Point pt;
        pt.x = current_x.load();
        pt.y = current_y.load();
        pt.z = current_z.load(); 
        pub_target->publish(pt);

        std_msgs::msg::Float32 yaw_msg;
        yaw_msg.data = current_yaw.load();
        pub_yaw->publish(yaw_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlungAntiswingWaypoint>());
    rclcpp::shutdown();
    return 0;
}