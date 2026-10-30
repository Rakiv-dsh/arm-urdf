#include "motor_ros2/motor_cfg.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic_bool interrupted{false};
void on_signal(int) { interrupted = true; }
}

int main(int argc, char **argv) {
  double duration = 0.0;
  double rate_hz = 10.0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--duration" && i + 1 < argc) {
      duration = std::stod(argv[++i]);
    } else if (arg == "--rate" && i + 1 < argc) {
      rate_hz = std::stod(argv[++i]);
    } else {
      std::cerr << "Usage: el05_joint2_current_monitor [--duration seconds] [--rate Hz]\n";
      return 2;
    }
  }
  if (duration < 0.0 || rate_hz <= 0.0 || rate_hz > 20.0) {
    std::cerr << "Limits: duration >= 0, 0 < rate <= 20 Hz\n";
    return 2;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  RobStrideMotor motor("can0", 0xFD, 2, 0);
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  const auto start = std::chrono::steady_clock::now();

  std::cout << "Monitoring motor 2 iqf (filtered q-axis current, A). Press Ctrl+C to stop.\n";
  std::cout << "elapsed_s,iqf_A\n";
  while (!interrupted) {
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    if (duration > 0.0 && elapsed >= duration)
      break;
    try {
      motor.Get_RobStrite_Motor_parameter(0x701A);
      const double current = motor.drw.iqf.data;
      if (!std::isfinite(current))
        throw std::runtime_error("invalid iqf feedback");
      std::cout << std::fixed << std::setprecision(3) << elapsed << ',' << current << '\n';
    } catch (const std::exception &error) {
      std::cerr << "Current query failed: " << error.what() << '\n';
    }
    std::this_thread::sleep_for(period);
  }
  std::cout << "Current monitor stopped; no motor command was sent.\n";
  return 0;
}
