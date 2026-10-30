#include "motor_ros2/motor_cfg.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
std::atomic_bool interrupted{false};
void on_signal(int) { interrupted = true; }
}

int main(int argc, char **argv) {
  bool confirmed = false;
  bool large_motion_confirmed = false;
  bool hold_enabled = false;
  double duration = 10.0;
  double amplitude = 0.005;
  double frequency = 0.1;
  double speed_limit = 0.1;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--confirm-hardware") {
      confirmed = true;
    } else if (arg == "--confirm-large-motion") {
      large_motion_confirmed = true;
    } else if (arg == "--hold-enabled") {
      hold_enabled = true;
    } else if (arg == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (arg == "--amplitude" && i + 1 < argc) {
      amplitude = std::stod(argv[++i]);
    } else if (arg == "--frequency" && i + 1 < argc) {
      frequency = std::stod(argv[++i]);
    } else if (arg == "--speed" && i + 1 < argc) {
      speed_limit = std::stod(argv[++i]);
    } else {
      std::cerr << "Usage: el05_joint1_sine --confirm-hardware [--confirm-large-motion] [--hold-enabled]\n"
                << "  [--duration seconds] [--amplitude rad] [--frequency Hz] [--speed rad/s]\n";
      return 2;
    }
  }
  if (!confirmed) {
    std::cerr << "Refusing to enable hardware. Add --confirm-hardware after verifying emergency stop.\n";
    return 2;
  }
  constexpr double normal_max_amplitude = 0.05;
  constexpr double large_motion_max_amplitude = 0.872665;  // 50 degrees
  if (amplitude > normal_max_amplitude && !large_motion_confirmed) {
    std::cerr << "Large amplitude requires --confirm-large-motion after checking clearance and emergency stop.\n";
    return 2;
  }
  const double max_amplitude = large_motion_confirmed
      ? large_motion_max_amplitude : normal_max_amplitude;
  if (!(amplitude > 0.0 && amplitude <= max_amplitude) ||
      !(frequency > 0.0 && frequency <= 0.2) ||
      !(speed_limit > 0.0 && speed_limit <= 0.2)) {
    std::cerr << "Limits: 0 < amplitude <= " << max_amplitude
              << " rad, 0 < frequency <= 0.2, 0 < speed <= 0.2\n";
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  RobStrideMotor motor("can0", 0xFD, 1, 0);
  try {
    // A valid parameter response is required before any enable/motion command.
    motor.Get_RobStrite_Motor_parameter(0x7019);
    const double center = motor.drw.mechPos.data;
    if (!std::isfinite(center))
      throw std::runtime_error("Invalid mechPos feedback");

    constexpr double period = 0.02;      // 50 Hz
    std::cout << "joint1 center=" << center << " rad; amplitude=" << amplitude
              << " rad; press Ctrl+C to stop\n";

    const auto start = std::chrono::steady_clock::now();
    while (!interrupted) {
      const double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
      if (t >= duration)
        break;
      const double target = center + amplitude * std::sin(2.0 * M_PI * frequency * t);
      motor.RobStrite_Motor_PosCSP_control(speed_limit, target);
      std::this_thread::sleep_for(std::chrono::duration<double>(period));
    }
    if (hold_enabled && !interrupted) {
      std::cout << "Motion complete; motor remains enabled at its center. Press Ctrl+C to disable.\n";
      while (!interrupted) {
        motor.RobStrite_Motor_PosCSP_control((float)speed_limit, (float)center);
        std::this_thread::sleep_for(std::chrono::duration<double>(period));
      }
    }
    motor.Disenable_Motor(0);
    std::cout << "Motor stopped.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "No safe feedback/control transaction: " << error.what() << "\n";
    try { motor.Disenable_Motor(0); } catch (...) {}
    return 1;
  }
}
