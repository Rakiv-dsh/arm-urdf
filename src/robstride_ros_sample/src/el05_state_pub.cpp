#include "motor_ros2/motor_cfg.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {
constexpr std::size_t kN = 5;
}

// el05_state_pub —— 把电机实测位置发布成 /joint_states，让 RViz 里的模型跟着真机动。
//
// 和 slider_servo 方向相反：
//   slider_servo : /joint_states(滑块) -> 电机
//   el05_state_pub: 电机 -> /joint_states -> robot_state_publisher -> RViz
//
// 用法：
//   ros2 run rs_motor_ros2 el05_state_pub --ros-args -p rate_hz:=20.0
class El05StatePub final : public rclcpp::Node {
public:
  El05StatePub() : Node("el05_state_pub") {
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    const auto ids = declare_parameter<std::vector<int64_t>>(
        "motor_ids", std::vector<int64_t>{21, 22, 23, 24, 25});
    offsets_ = declare_parameter<std::vector<double>>("mech_offset",
                                                      std::vector<double>{0, 0, 0, 0, 0});
    dirs_ = declare_parameter<std::vector<double>>("joint_direction",
                                                   std::vector<double>{1, 1, 1, 1, 1});
    gears_ = declare_parameter<std::vector<double>>("gear_ratio",
                                                    std::vector<double>{1, 1, 1, 1, 1});
    // 模型显示用的附加偏置（rad）：只影响 RViz 里模型的姿态，不动电机、不动 URDF。
    // 用来把"模型零位"挪到和真机一致的姿态，逐个关节试。
    model_offset_ = declare_parameter<std::vector<double>>(
        "model_offset", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    iface_ = declare_parameter<std::string>("can_interface", "can0");
    const double rate = declare_parameter<double>("rate_hz", 20.0);
    if (ids.size() != kN) throw std::runtime_error("motor_ids 需要 5 个值");

    for (std::size_t i = 0; i < kN; ++i) {
      motor_ids_[i] = static_cast<uint8_t>(ids[i]);
      motors_[i] = std::make_unique<RobStrideMotor>(iface_, 0xFD, motor_ids_[i], 0);
    }
    pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "发布电机实测角度到 /joint_states（%s，%d 台，%.0f Hz）",
                iface_.c_str(), static_cast<int>(kN), rate);
  }

private:
  void tick() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.resize(kN, 0.0);
    for (std::size_t i = 0; i < kN; ++i) {
      try {
        motors_[i]->Get_RobStrite_Motor_parameter(0x7019);
        const double mech = motors_[i]->drw.mechPos.data;
        msg.position[i] = std::isfinite(mech)
                              ? (mech - offsets_[i]) / (dirs_[i] * gears_[i])
                              : last_[i];
        if (i < model_offset_.size()) msg.position[i] += model_offset_[i];
      } catch (const std::exception &) {
        msg.position[i] = last_[i];   // 读失败就沿用上一次
      }
      last_[i] = msg.position[i];
    }
    pub_->publish(msg);
  }

  std::vector<std::string> joint_names_;
  std::array<uint8_t, kN> motor_ids_{};
  std::vector<double> offsets_, dirs_, gears_, model_offset_;
  std::array<std::unique_ptr<RobStrideMotor>, kN> motors_;
  std::array<double, kN> last_{};
  std::string iface_{"can0"};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<El05StatePub>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("el05_state_pub"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
