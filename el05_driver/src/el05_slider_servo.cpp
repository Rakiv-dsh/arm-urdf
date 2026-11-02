#include "motor_ros2/motor_cfg.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace {
constexpr std::size_t kJointCount = 5;
}

// 滑块伺服：把 /joint_states（来自 joint_state_publisher_gui）转换成五台电机的
// CSP 位置指令。要点：
//   1. 默认不使能，要发 /arm/enable=true 才动；
//   2. 使能那一刻把目标设成「当前位置」，不会因为滑块在 0 而突然弹过去；
//   3. 目标按 max_speed 斜坡跟随，滑块猛拉也不会一步到位；
//   4. /joint_states 断流超过 watchdog_ms 立即失能（拔线/关窗口自动停）。
class SliderServo final : public rclcpp::Node {
public:
  SliderServo() : Node("el05_slider_servo") {
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    const auto ids = declare_parameter<std::vector<int64_t>>(
        "motor_ids", std::vector<int64_t>{21, 22, 23, 24, 25});
    offsets_ = declare_parameter<std::vector<double>>(
        "mech_offset", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    dirs_ = declare_parameter<std::vector<double>>(
        "joint_direction", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0});
    // 连续关节（无机械限位，可以转整圈）：目标取「离当前位置最近的那一圈」，
    // 免得多转 360°。实测 joint1/3/4/5 都碰不到硬限位；joint2 未测，先也按连续处理。
    const auto cont = declare_parameter<std::vector<int64_t>>(
        "continuous", std::vector<int64_t>{1, 1, 1, 1, 1});
    gears_ = declare_parameter<std::vector<double>>(
        "gear_ratio", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0});
    lower_ = declare_parameter<std::vector<double>>(
        "lower_limit", std::vector<double>{-3.14, -0.72, -0.89, -3.14, -1.255});
    upper_ = declare_parameter<std::vector<double>>(
        "upper_limit", std::vector<double>{3.14, 2.19, 2.34, 3.14, 1.75});
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    max_speed_ = declare_parameter<double>("max_speed", 0.3);     // rad/s，斜坡上限
    speed_limit_ = declare_parameter<double>("speed_limit", 0.5); // 写进电机 0x7017
    watchdog_ms_ = declare_parameter<double>("watchdog_ms", 1000.0);
    iface_ = declare_parameter<std::string>("can_interface", "can0");
    limit_cur_ = declare_parameter<double>("limit_cur", 0.0);  // >0 时使能前写 0x2019（限流 A）
    // 逗号分隔的关节名；空 = 全部驱动
    const auto active_str = declare_parameter<std::string>("active_joints", "");

    auto need = [this](const std::vector<double> &v, const char *name) {
      if (v.size() != kJointCount) {
        RCLCPP_ERROR(get_logger(), "%s 需要 %zu 个值，退出", name, kJointCount);
        throw std::runtime_error(name);
      }
    };
    need(offsets_, "mech_offset"); need(dirs_, "joint_direction");
    for (std::size_t i = 0; i < kJointCount && i < cont.size(); ++i) continuous_[i] = cont[i] != 0;
    need(gears_, "gear_ratio"); need(lower_, "lower_limit"); need(upper_, "upper_limit");
    if (ids.size() != kJointCount) { RCLCPP_ERROR(get_logger(), "motor_ids 需要 5 个值，退出"); throw std::runtime_error("motor_ids"); }

    for (std::size_t i = 0; i < kJointCount; ++i)
      motor_ids_[i] = static_cast<uint8_t>(ids[i]);

    // 只驱动 active_joints 里列出的关节；没列出的电机完全不碰（不使能、不发帧）
    std::vector<std::string> active;
    {
      std::string cur;
      for (char c : active_str) {
        if (c == ',') { if (!cur.empty()) active.push_back(cur); cur.clear(); }
        else if (c != ' ') cur.push_back(c);
      }
      if (!cur.empty()) active.push_back(cur);
    }
    for (std::size_t i = 0; i < kJointCount; ++i) active_[i] = active.empty();
    for (const auto &n : active) {
      bool hit = false;
      for (std::size_t i = 0; i < kJointCount; ++i)
        if (n == joint_names_[i]) { active_[i] = true; hit = true; }
      if (!hit) RCLCPP_WARN(get_logger(), "active_joints 里的 %s 不在 joint_names 里，忽略", n.c_str());
    }
    for (std::size_t i = 0; i < kJointCount; ++i)
      RCLCPP_INFO(get_logger(), "关节 %zu (%s, 电机 %d): %s", i, joint_names_[i].c_str(),
                  motor_ids_[i], active_[i] ? "驱动" : "不驱动");

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(*msg); });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/arm/enable", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { on_enable(msg->data); });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "滑块伺服就绪（输出关闭）。滑块在 /joint_states，发 /arm/enable=true 才开始驱动；"
                "电机 %s 上的 %d~%d，max_speed=%.2f rad/s，看门狗 %.0f ms",
                iface_.c_str(), motor_ids_.front(), motor_ids_.back(), max_speed_, watchdog_ms_);
  }

  ~SliderServo() override { stop_all(); }

private:
  static double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

  void on_joint_state(const sensor_msgs::msg::JointState &msg) {
    bool any = false;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      for (std::size_t j = 0; j < msg.name.size() && j < msg.position.size(); ++j) {
        if (msg.name[j] == joint_names_[i] && std::isfinite(msg.position[j])) {
          desired_joint_[i] = msg.position[j];
          any = true;
        }
      }
    }
    if (any) { have_slider_ = true; last_msg_ = std::chrono::steady_clock::now(); }
  }

  void on_enable(bool on) {
    if (on) {
      if (enabled_) return;
      if (!have_slider_) { RCLCPP_WARN(get_logger(), "还没收到 /joint_states，先不动"); return; }
      try {
        for (std::size_t i = 0; i < kJointCount; ++i) {
          if (!active_[i]) continue;
          motors_[i] = std::make_unique<RobStrideMotor>(iface_, 0xFD, motor_ids_[i], 0);
          motors_[i]->Get_RobStrite_Motor_parameter(0x7019);
          const double mech = motors_[i]->drw.mechPos.data;
          if (!std::isfinite(mech)) throw std::runtime_error("mechPos 无效");
          // 当前位置当起点：滑块在哪儿都先不跳
          target_[i] = mech;
          if (limit_cur_ > 0.0) {   // 降电流限幅，避免堵转时把母线拉垮
            motors_[i]->Set_RobStrite_Motor_parameter(0x2019, static_cast<float>(limit_cur_),
                                                     Set_parameter);
            RCLCPP_INFO(get_logger(), "  joint %zu 电流限幅设为 %.1f A", i, limit_cur_);
          }
          // 起点对齐：目标=当前位置，滑块位置暂存但不立即生效（斜坡自然会追上）
          desired_joint_[i] = (mech - offsets_[i]) / (dirs_[i] * gears_[i]);
          RCLCPP_INFO(get_logger(), "joint %zu (can %d) 起始机械角 %.4f rad", i,
                      motor_ids_[i], mech);
        }
        // 首次下发：切到 CSP 模式 + 写限速 + 写一次目标
        for (std::size_t i = 0; i < kJointCount; ++i)
          if (active_[i])
            motors_[i]->RobStrite_Motor_PosCSP_control(
                static_cast<float>(speed_limit_), static_cast<float>(target_[i]));
        last_tick_ = std::chrono::steady_clock::now();
        enabled_ = true;
        last_msg_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "已使能，开始跟随滑块");
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "使能失败：%s", e.what());
        stop_all();
      }
    } else {
      RCLCPP_INFO(get_logger(), "收到 /arm/enable=false，失能");
      stop_all();
    }
  }

  void tick() {
    if (!enabled_) return;
    const auto now = std::chrono::steady_clock::now();
    const double silent_ms =
        std::chrono::duration<double, std::milli>(now - last_msg_).count();
    if (watchdog_ms_ > 0.0 && silent_ms > watchdog_ms_) {
      RCLCPP_WARN(get_logger(), "/joint_states 断了 %.0f ms，失能保安全", silent_ms);
      stop_all();
      return;
    }
    // 用真实经过时间做斜坡，循环实际跑多少 Hz 都不影响速度
    double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    dt = clampd(dt, 0.0, 0.1);
    try {
      for (std::size_t i = 0; i < kJointCount; ++i) {
        if (!active_[i]) continue;
        // 关节目标 → 夹到限位 → 换算成电机侧 → 斜坡
        // 连续轴（无机械限位）不受 ±180° 夹取，否则拖过 180° 就卡住不动；
        // 有限位的轴照旧夹。
        const double q = continuous_[i]
                             ? desired_joint_[i]
                             : clampd(desired_joint_[i], lower_[i], upper_[i]);
        double want = offsets_[i] + dirs_[i] * gears_[i] * q;
        if (continuous_[i]) {
          // 搬到离当前目标最近的一圈：|want-target| 不会超过半圈
          want += 2.0 * M_PI * std::round((target_[i] - want) / (2.0 * M_PI));
        }
        const double step = max_speed_ * dt;
        target_[i] += clampd(want - target_[i], -step, step);
        // 模式与限速已在使能时写过，这里每周期只写目标位置（0x7016），CAN 负载减半
        motors_[i]->Set_RobStrite_Motor_parameter(0x7016, static_cast<float>(target_[i]),
                                                 Set_parameter);
        if (motors_[i]->has_fault()) {
          RCLCPP_ERROR(get_logger(), "joint %zu 报故障，全部失能", i);
          stop_all();
          return;
        }
      }
      fail_streak_ = 0;
      if (++log_tick_ >= static_cast<int>(std::max(1.0, control_rate_hz_))) {
        log_tick_ = 0;
        for (std::size_t i = 0; i < kJointCount; ++i) {
          if (!active_[i] || !motors_[i]) continue;
          RCLCPP_INFO(get_logger(), "%s: 滑块 %.3f rad -> 目标 %.3f（电机侧 %.3f）实测 %.3f",
                      joint_names_[i].c_str(), desired_joint_[i],
                      (target_[i] - offsets_[i]) / (dirs_[i] * gears_[i]), target_[i],
                      motors_[i]->position_);
        }
      }
    } catch (const std::exception &e) {
      if (++fail_streak_ >= 5) {
        RCLCPP_ERROR(get_logger(), "CAN 连续 %d 次失败（%s），全部失能", fail_streak_, e.what());
        stop_all();
      }
    }
  }

  void stop_all() {
    for (auto &m : motors_) {
      if (m) { try { m->Disenable_Motor(0); } catch (...) {} }
    }
    for (auto &m : motors_) m.reset();
    enabled_ = false;
  }

  std::vector<std::string> joint_names_;
  std::array<uint8_t, kJointCount> motor_ids_{};
  std::vector<double> offsets_, dirs_, gears_, lower_, upper_;
  std::array<std::unique_ptr<RobStrideMotor>, kJointCount> motors_;
  std::array<double, kJointCount> desired_joint_{}, target_{};
  std::string iface_{"can0"};
  double control_rate_hz_{50.0}, max_speed_{0.3}, speed_limit_{0.5}, watchdog_ms_{1000.0}, limit_cur_{0.0};
  bool have_slider_{false}, enabled_{false};
  std::array<bool, kJointCount> active_{};
  std::array<bool, kJointCount> continuous_{};
  int fail_streak_{0};
  int log_tick_{0};
  std::chrono::steady_clock::time_point last_msg_{};
  std::chrono::steady_clock::time_point last_tick_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SliderServo>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("el05_slider_servo"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
