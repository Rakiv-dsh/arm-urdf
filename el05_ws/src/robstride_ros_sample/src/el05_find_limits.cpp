#include "motor_ros2/motor_cfg.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

// el05_find_limits —— 单向自动探限：一点一点往一个方向走，直到"走不动"就记下来。
//
// 关键：每走一步都要等电机真的到位（等它追上目标），再判断这一步有没有走成。
// 否则目标跑在前面，看起来"停在半路"，实际上限位还没到。
//
// 用法：
//   ros2 run rs_motor_ros2 el05_find_limits --confirm-hardware --motor 22 --dir +1
//   ... --step 0.03 --speed 0.12 --current 4 --max-travel 2.0 --stall 4
//
// 退出码：0=找到限位  1=没找到（走满 max-travel 或异常）

int main(int argc, char **argv) {
  bool confirmed = false;
  int motor_id = 21, dir = 1, stall_need = 4;
  double step = 0.03, speed = 0.12, current = 4.0, max_travel = 2.0, settle_ms = 900.0;
  std::string iface = "can0";

  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    if (a == "--confirm-hardware") confirmed = true;
    else if (a == "--motor" && i + 1 < argc) motor_id = std::stoi(argv[++i]);
    else if (a == "--dir" && i + 1 < argc) dir = std::stoi(argv[++i]) >= 0 ? 1 : -1;
    else if (a == "--step" && i + 1 < argc) step = std::stod(argv[++i]);
    else if (a == "--speed" && i + 1 < argc) speed = std::stod(argv[++i]);
    else if (a == "--current" && i + 1 < argc) current = std::stod(argv[++i]);
    else if (a == "--max-travel" && i + 1 < argc) max_travel = std::stod(argv[++i]);
    else if (a == "--stall" && i + 1 < argc) stall_need = std::stoi(argv[++i]);
    else if (a == "--iface" && i + 1 < argc) iface = argv[++i];
    else {
      std::cerr << "用法: el05_find_limits --confirm-hardware --motor N --dir +1|-1"
                   " [--step rad] [--speed rad/s] [--current A] [--max-travel rad] [--stall N]\n";
      return 2;
    }
  }
  if (!confirmed) {
    std::cerr << "需要 --confirm-hardware（确认限位附近没人手、没线缆、留了空间）\n";
    return 2;
  }
  if (step <= 0 || step > 0.1 || speed <= 0 || speed > 0.3 || max_travel <= 0) {
    std::cerr << "参数超范围：0<step<=0.1，0<speed<=0.3，max_travel>0\n";
    return 2;
  }

  try {
    RobStrideMotor motor(iface, 0xFD, static_cast<uint8_t>(motor_id), 0);
    motor.Get_RobStrite_Motor_parameter(0x7019);
    double pos = motor.drw.mechPos.data;
    if (!std::isfinite(pos)) {
      std::cerr << "[limits] 读不到电机 " << motor_id << " 的位置\n";
      return 1;
    }
    if (current > 0) {
      motor.Set_RobStrite_Motor_parameter(0x2019, static_cast<float>(current), Set_parameter);
    }

    const double start = pos;
    double target = pos;
    int stall = 0;
    std::cout << "[limits] 电机 " << motor_id << " 起点 " << pos << " rad，方向 "
              << (dir > 0 ? "+" : "-") << "，步长 " << step << " rad，速度 " << speed << " rad/s\n";

    while (true) {
      target += dir * step;
      // 等这一步真的走完（电机追上目标），最多 settle_ms
      bool settled = false;
      const auto t0 = std::chrono::steady_clock::now();
      while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() * 1000.0 < settle_ms) {
        auto fb = motor.RobStrite_Motor_PosCSP_control(static_cast<float>(speed),
                                                       static_cast<float>(target));
        pos = std::get<0>(fb);
        if (!std::isfinite(pos)) break;
        if (std::fabs(pos - target) < step * 0.35) { settled = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
      if (motor.has_fault()) {
        std::cout << "[limits] 电机报故障，停在 " << pos << " rad\n";
        break;
      }
      if (!settled) {
        ++stall;
        std::cout << "   走不动 " << stall << "/" << stall_need << "（位置 " << pos
                  << "，差 " << (target - pos) << "）\n";
        if (stall >= stall_need) {
          for (int b = 0; b < 3; ++b) {
            target -= dir * step;
            motor.RobStrite_Motor_PosCSP_control(static_cast<float>(speed),
                                                 static_cast<float>(target));
            std::this_thread::sleep_for(std::chrono::milliseconds(120));
          }
          auto fb = motor.RobStrite_Motor_PosCSP_control(static_cast<float>(speed),
                                                          static_cast<float>(target));
          std::cout << "[limits] 找到限位：约 " << pos << " rad（从起点走 "
                    << (pos - start) << " rad），已退回 " << std::get<0>(fb) << " rad\n";
          return 0;
        }
      } else {
        stall = 0;
      }
      if (std::fabs(pos - start) >= max_travel) {
        std::cout << "[limits] 走满 " << max_travel << " rad 没碰到限位，停在 " << pos << " rad\n";
        return 1;
      }
    }
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "[limits] 异常：" << e.what() << "\n";
    return 1;
  }
}
