#include "motor_ros2/motor_cfg.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// el05_home —— 把各轴开回关节角 0（电机内部零位）。
// 顺序默认从远端到根部，避免底座先摆；每轴走完再走下一轴，走的时候一直发目标。
//
// 用法：
//   ros2 run rs_motor_ros2 el05_home --confirm-hardware
//   ros2 run rs_motor_ros2 el05_home --confirm-hardware --order 25,24,23,22,21 --speed 0.05
//   ros2 run rs_motor_ros2 el05_home --confirm-hardware --tol 0.02 --timeout 20 --current 6

namespace {
std::atomic_bool stop_flag{false};
void on_signal(int) { stop_flag = true; }
}  // namespace

int main(int argc, char **argv) {
  bool confirmed = false;
  double tol = 0.02, speed = 0.05, timeout = 20.0, current = 6.0, big = 1.2;
  std::vector<int> order = {25, 24, 23, 22, 21};
  std::string iface = "can0";

  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    if (a == "--confirm-hardware") confirmed = true;
    else if (a == "--tol" && i + 1 < argc) tol = std::stod(argv[++i]);
    else if (a == "--speed" && i + 1 < argc) speed = std::stod(argv[++i]);
    else if (a == "--timeout" && i + 1 < argc) timeout = std::stod(argv[++i]);
    else if (a == "--current" && i + 1 < argc) current = std::stod(argv[++i]);
    else if (a == "--big" && i + 1 < argc) big = std::stod(argv[++i]);
    else if (a == "--iface" && i + 1 < argc) iface = argv[++i];
    else if (a == "--order" && i + 1 < argc) {
      order.clear();
      std::string s(argv[++i]), cur;
      for (char c : s) {
        if (c == ',') { if (!cur.empty()) order.push_back(std::stoi(cur)); cur.clear(); }
        else cur.push_back(c);
      }
      if (!cur.empty()) order.push_back(std::stoi(cur));
    } else {
      std::cerr << "用法: el05_home --confirm-hardware [--order 25,24,23,22,21]"
                   " [--speed rad/s] [--tol rad] [--timeout s] [--current A] [--iface can0]\n";
      return 2;
    }
  }
  if (!confirmed) {
    std::cerr << "需要 --confirm-hardware（确认机械臂周围安全、已有人扶/垫）\n";
    return 2;
  }
  if (speed <= 0 || speed > 0.5 || tol <= 0 || tol > 0.5) {
    std::cerr << "参数超范围：0<speed<=0.5，0<tol<=0.5\n";
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::cout << "[home] 顺序";
  for (int id : order) std::cout << " " << id;
  std::cout << "；速度 " << speed << " rad/s，容差 " << tol << " rad，限流 " << current << " A\n";

  int failed = 0;
  for (int id : order) {
    if (stop_flag) break;
    try {
      RobStrideMotor motor(iface, 0xFD, static_cast<uint8_t>(id), 0);
      // 注意：不要用 read_initial_position()——它内部把电机号写死成 1 号、且用阻塞 read()
      // 等一帧永远不会来的回包，会卡死。直接读参数拿机械角。
      motor.Get_RobStrite_Motor_parameter(0x7019);
      const double q = static_cast<double>(motor.drw.mechPos.data);
      if (!std::isfinite(q)) {
        std::cout << "[home] 电机 " << id << " 位置无效，跳过\n";
        ++failed;
        continue;
      }
      std::cout << "[home] 电机 " << id << " 当前 " << q << " rad";
      if (std::fabs(q) <= tol) {
        std::cout << "，已在零位，跳过\n";
        continue;
      }
      if (std::fabs(q) > big) {
        std::cout << "，偏差过大（>" << big << "），跳过（确认安全后用 --big 放宽）\n";
        ++failed;
        continue;
      }
      std::cout << "，开始回零\n";
      if (current > 0) {
        motor.Set_RobStrite_Motor_parameter(0x2019, static_cast<float>(current), Set_parameter);
      }
      const auto t0 = std::chrono::steady_clock::now();
      bool reached = false;
      while (!stop_flag) {
        const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (el > timeout) break;
        auto fb = motor.RobStrite_Motor_PosCSP_control(static_cast<float>(speed), 0.0f);
        const double pos = std::get<0>(fb);
        if (std::fabs(pos) <= tol) { reached = true; break; }
        if (motor.has_fault()) {
          std::cerr << "[home] 电机 " << id << " 报故障，停止\n";
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (reached) {
        std::cout << "[home] 电机 " << id << " 到位\n";
      } else {
        std::cout << "[home] 电机 " << id << " 超时未到位，保持当前\n";
        ++failed;
      }
    } catch (const std::exception &e) {
      std::cerr << "[home] 电机 " << id << " 异常：" << e.what() << "\n";
      ++failed;
    }
  }

  std::cout << "[home] 结束" << (failed ? "（有未到位轴）" : "（全部到位）") << "\n";
  return failed ? 1 : 0;
}
