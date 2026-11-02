#include "motor_ros2/motor_cfg.h"
#include "stdint.h"
#include <atomic>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>
#include <unistd.h>
#include <vector>

class MotorControlSample : public rclcpp::Node {
public:
  MotorControlSample()
      : rclcpp::Node("motor_control_set_node"),
        motor_can0(RobStrideMotor("can0", 0xFD, 0x01, 0)),
        motor_can4(RobStrideMotor("can4", 0xFD, 0x05, 0)) {

    motor_can0.Get_RobStrite_Motor_parameter(0x7005);
    motor_can4.Get_RobStrite_Motor_parameter(0x7005);
    usleep(1000);
    motor_can0.enable_motor();
    motor_can4.enable_motor();
    can0_position_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/motor_can0/position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          can0_position_ = msg->data;
          can0_position_received_ = true;
        });
    can4_position_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/motor_can4/position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          can4_position_ = msg->data;
          can4_position_received_ = true;
        });
    usleep(1000);
    worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);
  }

  ~MotorControlSample() {
    motor_can0.Disenable_Motor(0);
    motor_can4.Disenable_Motor(0);
    running_ = false; // 停止线程
    if (worker_thread_.joinable())
      worker_thread_.join(); // 等待线程结束
  }

  void excute_loop() {
    while (running_) {
      if (can0_position_received_)
        motor_can0.RobStrite_Motor_PosPP_control(1.0f, 0.5f, can0_position_.load());
      if (can4_position_received_)
        motor_can4.RobStrite_Motor_PosPP_control(1.0f, 0.5f, can4_position_.load());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  std::thread worker_thread_;
  std::atomic<bool> running_ = true;

  RobStrideMotor motor_can0;
  RobStrideMotor motor_can4;
  std::atomic<float> can0_position_{0.0f};
  std::atomic<float> can4_position_{0.0f};
  std::atomic<bool> can0_position_received_{false};
  std::atomic<bool> can4_position_received_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr can0_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr can4_position_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorControlSample>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
