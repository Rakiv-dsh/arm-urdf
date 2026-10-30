#include "motor_ros2/motor_cfg.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace {
constexpr std::size_t kJointCount = 5;
constexpr double kPi = 3.14159265358979323846;
}

class FiveDofArmController final : public rclcpp::Node {
public:
  FiveDofArmController() : Node("five_dof_arm_controller") {
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    // All five EL05 motors share one physical CAN bus exposed as can0.
    interfaces_ = {"can0", "can0", "can0", "can0", "can0"};
    motor_ids_ = {1, 2, 3, 4, 5};
    // Mechanical offsets for CAN IDs 1~5, in radians.
    offsets_ = {0.0, 0.265762, 0.0, 4.244908, 0.0};
    directions_ = {1.0, 1.0, 1.0, 1.0, 1.0};
    gear_ratios_ = {1.0, 1.0, 1.0, 1.0, 1.0};
    lower_ = {-3.14, -0.720, -0.890, -3.14, -1.255};
    upper_ = {3.14, 2.190, 2.340, 3.14, 1.750};
    velocity_limits_ = {1.0, 1.0, 1.0, 1.0, 1.0};
    q_target_.fill(0.0);
    q_measured_.fill(0.0);

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    position_kp_ = declare_parameter<double>("position_kp", 2.0);
    sine_amplitude_ = declare_parameter<double>("sine_amplitude", 0.05);
    sine_frequency_ = declare_parameter<double>("sine_frequency", 0.1);
    max_temperature_ = declare_parameter<double>("max_temperature", 80.0);
    max_torque_ = declare_parameter<double>("max_torque", 10.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
    load_vector_parameter("mech_offset", offsets_);
    load_vector_parameter("joint_direction", directions_);
    load_vector_parameter("gear_ratio", gear_ratios_);
    load_vector_parameter("lower_limit", lower_);
    load_vector_parameter("upper_limit", upper_);
    load_vector_parameter("velocity_limit", velocity_limits_);

    for (std::size_t i = 0; i < kJointCount; ++i) {
      motors_[i] = std::make_unique<RobStrideMotor>(interfaces_[i], 0xFD,
                                                     motor_ids_[i], 0);
    }

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/arm/joint_command", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          for (std::size_t i = 0; i < kJointCount; ++i) {
            for (std::size_t j = 0; j < msg->name.size() && j < msg->position.size(); ++j) {
              if (msg->name[j] == joint_names_[i] && std::isfinite(msg->position[j])) {
                q_target_[i] = clamp(msg->position[j], lower_[i], upper_[i]);
                command_received_ = true;
                last_command_ = std::chrono::steady_clock::now();
              }
            }
          }
        });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/arm/enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          enabled_ = msg->data;
          if (enabled_)
            motors_initialized_for_stop_ = true;
          else
            stop_motors();
        });
    sine_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/arm/sine_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          sine_enabled_ = msg->data;
          if (sine_enabled_) {
            sine_start_ = std::chrono::steady_clock::now();
            command_received_ = true;
          }
        });
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "/arm/joint_states", 10);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_)));
    timer_ = create_wall_timer(period, [this]() { control_step(); });
    RCLCPP_INFO(get_logger(),
                "Five-DOF controller ready; output disabled. Publish /arm/enable=true to start.");
  }

  ~FiveDofArmController() override { stop_motors(); }

private:
  static double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
  }

  template <typename Array>
  void load_vector_parameter(const std::string &name, Array &target) {
    const auto values = declare_parameter<std::vector<double>>(name, std::vector<double>{});
    if (values.size() != target.size()) {
      RCLCPP_WARN(get_logger(), "%s must contain %zu values; keeping defaults",
                  name.c_str(), target.size());
      return;
    }
    std::copy(values.begin(), values.end(), target.begin());
  }

  void control_step() {
    if (!enabled_ || !command_received_) {
      publish_state();
      return;
    }
    if (!sine_enabled_ && command_timeout_ > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_).count() >
            command_timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Joint command timeout; stopping motors");
      stop_motors();
      return;
    }

    std::array<double, kJointCount> desired = q_target_;
    if (sine_enabled_) {
      const double t = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - sine_start_)
                           .count();
      const double phase = 2.0 * kPi * sine_frequency_ * t;
      for (std::size_t i = 0; i < kJointCount; ++i)
        desired[i] = clamp(sine_amplitude_ * std::sin(phase), lower_[i], upper_[i]);
    }

    try {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        const double error = desired[i] - q_measured_[i];
        const double joint_velocity = clamp(position_kp_ * error,
                                            -velocity_limits_[i], velocity_limits_[i]);
        const double motor_velocity = directions_[i] * gear_ratios_[i] * joint_velocity;
        const auto feedback = motors_[i]->send_velocity_mode_command(
            static_cast<float>(motor_velocity));
        q_measured_[i] = directions_[i] *
                         (static_cast<double>(std::get<0>(feedback)) - offsets_[i]) /
                         gear_ratios_[i];
        if (q_measured_[i] < lower_[i] - 0.05 || q_measured_[i] > upper_[i] + 0.05 ||
            std::abs(std::get<2>(feedback)) > max_torque_ ||
            std::get<3>(feedback) > max_temperature_ || motors_[i]->has_fault()) {
          RCLCPP_ERROR(get_logger(), "Safety stop on %s", joint_names_[i].c_str());
          stop_motors();
          return;
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "CAN/control error: %s; stopping all motors", error.what());
      stop_motors();
      return;
    }
    publish_state();
  }

  void publish_state() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.assign(q_measured_.begin(), q_measured_.end());
    state_pub_->publish(msg);
  }

  void stop_motors() {
    if (!enabled_ && !motors_initialized_for_stop_)
      return;
    for (auto &motor : motors_) {
      if (motor) {
        try {
          motor->Disenable_Motor(0);
        } catch (...) {
          // Continue stopping the remaining channels.
        }
      }
    }
    enabled_ = false;
    command_received_ = false;
    motors_initialized_for_stop_ = false;
  }

  std::array<std::unique_ptr<RobStrideMotor>, kJointCount> motors_;
  std::array<std::string, kJointCount> joint_names_;
  std::array<std::string, kJointCount> interfaces_;
  std::array<uint8_t, kJointCount> motor_ids_;
  std::array<double, kJointCount> offsets_;
  std::array<double, kJointCount> directions_;
  std::array<double, kJointCount> gear_ratios_;
  std::array<double, kJointCount> lower_;
  std::array<double, kJointCount> upper_;
  std::array<double, kJointCount> velocity_limits_;
  std::array<double, kJointCount> q_target_;
  std::array<double, kJointCount> q_measured_;
  double control_rate_hz_ = 50.0;
  double position_kp_ = 2.0;
  double sine_amplitude_ = 0.05;
  double sine_frequency_ = 0.1;
  double max_temperature_ = 80.0;
  double max_torque_ = 10.0;
  double command_timeout_ = 0.5;
  bool enabled_ = false;
  bool sine_enabled_ = false;
  bool command_received_ = false;
  bool motors_initialized_for_stop_ = true;
  std::chrono::steady_clock::time_point sine_start_;
  std::chrono::steady_clock::time_point last_command_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sine_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FiveDofArmController>());
  rclcpp::shutdown();
  return 0;
}
