// 状态：已弃用
// ==============================================================================
// @File     : slung_takeoff.cpp (或 offboard_control.cpp)
// @Author   : 4Shuan9
// @Date     : 2026-07-31
// @Brief    : 基础 Offboard 起飞悬停测试节点
// 
// @Main Work: 
//   * 原始功能：发布 Offboard 心跳，发送解锁指令并控制无人机平滑爬升至指定高度悬停
//   * 冗余说明：核心的起飞、解锁、预热和防摆阶段切换逻辑已经全部被集成到最顶层的 `slung_antiswing.cpp` 节点中
//             当前该文件已不再参与防摆控制闭环
//   * 处理建议：可作为 PX4 Offboard 控制的最简 C++ 参考模板保留，或直接从项目中删除以保持代码库整洁
// ==============================================================================

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <atomic>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control") {
        // 1. 初始化发布器
        pub_offboard_mode_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        pub_trajectory_    = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        pub_vehicle_cmd_   = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        
        // 2. 订阅无人机实时位置
        sub_local_pos_ = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
            std::bind(&OffboardControl::local_pos_cb, this, std::placeholders::_1));

        auto timer_cb = [this]() -> void {
            // 每 1 秒打印一次高度信息
            if (++sec_counter_ >= 10) {
                RCLCPP_INFO(this->get_logger(), "Current Z: %.2f m | Target Z: %.2f m", 
                            -(current_z_.load()), -target_z_);
                sec_counter_ = 0;
            }
            
            // 预热 10 个周期后：切 Offboard 模式、解锁并记录起飞原点
            if (setpoint_counter_ == 10) {
                publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
                
                target_x_ = current_x_.load();
                target_y_ = current_y_.load();
                target_z_ = current_z_.load();
            }

            // 规划平滑爬升轨迹
            if (setpoint_counter_ >= 10) {
                if (target_z_ > FINAL_Z) { 
                    target_z_ -= (ASCENT_SPEED * DT); 
                } else {
                    target_z_ = FINAL_Z; 
                }
            }

            // 持续发送心跳与控制指令
            publish_offboard_mode();
            publish_trajectory();

            if (setpoint_counter_ < 11) {
                setpoint_counter_++;
            }
        };

        // 3. 开启 10Hz 控制定时器
        timer_ = this->create_wall_timer(100ms, timer_cb);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr pub_offboard_mode_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr pub_vehicle_cmd_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr sub_local_pos_;

    uint8_t setpoint_counter_ = 0;
    uint8_t sec_counter_ = 0;
    
    // 实时坐标 (原子操作保证线程安全)
    std::atomic<float> current_x_{0.0f};
    std::atomic<float> current_y_{0.0f};
    std::atomic<float> current_z_{0.0f};

    // 目标坐标
    float target_x_{0.0f};
    float target_y_{0.0f};
    float target_z_{0.0f};
    
    // 轨迹参数配置
    const float FINAL_Z = -5.0f;     // 目标高度 (NED坐标系，负值为上，单位: m)
    const float ASCENT_SPEED = 0.5f; // 爬升速度 (m/s)
    const float DT = 0.1f;           // 控制周期 (100ms)

    // 发送解锁命令
    void arm() {
        publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Armed");
    }

    // 发送 Offboard 控制心跳
    void publish_offboard_mode() {
        OffboardControlMode msg{};
        msg.position = true;    // 启用位置控制
        msg.velocity = true;    // 启用速度前馈
        msg.acceleration = false;                                           
        msg.attitude = false;                                               
        msg.body_rate = false;                                              
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;      
        pub_offboard_mode_->publish(msg);
    }

    // 发送期望轨迹点
    void publish_trajectory() {
        TrajectorySetpoint msg{};
        
        // 设置位置期望点
        msg.position = {target_x_, target_y_, target_z_};
        
        // 设置速度前馈 (爬升阶段给予向上的 Z 轴速度)
        if (target_z_ > FINAL_Z) {
            msg.velocity = {0.0f, 0.0f, -ASCENT_SPEED}; 
        } else {
            msg.velocity = {0.0f, 0.0f, 0.0f};
        }

        msg.yaw = 1.57f; // 锁定偏航角 (约 90 度)
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub_trajectory_->publish(msg);
    }

    // 发送 Vehicle Command 基础封装
    void publish_vehicle_cmd(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        pub_vehicle_cmd_->publish(msg);
    }

    // 实时位置订阅回调
    void local_pos_cb(const VehicleLocalPosition::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_z_ = msg->z;
    }
};

int main(int argc, char *argv[]) {
    std::cout << "Starting smooth offboard control node..." << std::endl;
    // 取消输出缓冲以确保日志即时打印
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}