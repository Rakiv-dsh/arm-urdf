#include "motor_ros2/motor_cfg.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
namespace { std::atomic_bool stop{false}; void sig(int){stop=true;} }
int main(int argc,char **argv){
  bool ok=false, large_motion_confirmed=false, extreme_motion_confirmed=false, hold_enabled=false, monitor_id2_current=false; double duration=60, amp=.02, amp_id2=-1.0, freq=.1, speed=.1, speed_id2=-1.0;
  std::array<bool,5> active{}; active.fill(true);
  for(int i=1;i<argc;i++){std::string a=argv[i];
    if(a=="--confirm-hardware") ok=true;
    else if(a=="--confirm-large-motion") large_motion_confirmed=true;
    else if(a=="--confirm-extreme-motion") extreme_motion_confirmed=true;
    else if(a=="--hold-enabled") hold_enabled=true;
    else if(a=="--monitor-id2-current") monitor_id2_current=true;
    else if(i+1<argc && a=="--duration") duration=std::stod(argv[++i]);
    else if(i+1<argc && a=="--amplitude") amp=std::stod(argv[++i]);
    else if(i+1<argc && a=="--amplitude-id2") amp_id2=std::stod(argv[++i]);
    else if(i+1<argc && a=="--frequency") freq=std::stod(argv[++i]);
    else if(i+1<argc && a=="--speed") speed=std::stod(argv[++i]);
    else if(i+1<argc && a=="--speed-id2") speed_id2=std::stod(argv[++i]);
    else if(i+1<argc && a=="--skip-id") {
      int id=std::stoi(argv[++i]);
      if(id<1 || id>5){std::cerr<<"--skip-id must be between 1 and 5.\n"; return 2;}
      active[(size_t)(id-1)]=false;
    }
    else {std::cerr<<"Usage: el05_all_sine --confirm-hardware [--confirm-large-motion] [--confirm-extreme-motion] [--hold-enabled] [--monitor-id2-current] [--skip-id N] [--duration s] [--amplitude rad] [--amplitude-id2 rad] [--frequency Hz] [--speed rad/s] [--speed-id2 rad/s]\n"; return 2;}}
  constexpr double normal_max_amplitude = .05;
  constexpr double large_motion_max_amplitude = .872665; // 50 degrees
  constexpr double extreme_motion_max_amplitude = 1.570796; // 90 degrees
  if (amp_id2 > large_motion_max_amplitude && !extreme_motion_confirmed) {
    std::cerr << "Amplitudes above 50 degrees require --confirm-extreme-motion.\n";
    return 2;
  }
  if(amp > normal_max_amplitude && !large_motion_confirmed){
    std::cerr<<"Large amplitude requires --confirm-large-motion after checking clearance and emergency stop.\n";
    return 2;
  }
  const double max_amp = extreme_motion_confirmed ? extreme_motion_max_amplitude : (large_motion_confirmed ? large_motion_max_amplitude : normal_max_amplitude);
  if (amp_id2 < 0.0) amp_id2 = amp;
  constexpr double normal_max_speed = .2;
  constexpr double large_motion_max_speed = 5.235988; // 50 rpm
  if (speed_id2 < 0.0) speed_id2 = speed;
  const double max_speed_id2 = large_motion_confirmed ? large_motion_max_speed : normal_max_speed;
  if(!ok||duration<=0||amp<=0||amp>max_amp||amp_id2<=0||amp_id2>max_amp||freq<=0||freq>.2||speed<=0||speed>normal_max_speed||speed_id2<=0||speed_id2>max_speed_id2){
    std::cerr<<"Refusing run: confirmation required; limits amp<="<<max_amp<<" rad, freq<=.2, speed<=.2.\n";
    return 2;
  }
  std::signal(SIGINT,sig); std::signal(SIGTERM,sig);
  std::array<std::unique_ptr<RobStrideMotor>,5> m; std::array<double,5> c{};
  if (monitor_id2_current && !active[1]) { std::cerr << "Cannot monitor skipped motor ID 2.\n"; return 2; }
  try { for(int i=0;i<5;i++){if(!active[i]) continue; m[i]=std::make_unique<RobStrideMotor>("can0",0xFD,(uint8_t)(i+1),0); m[i]->Get_RobStrite_Motor_parameter(0x7019); c[i]=m[i]->drw.mechPos.data; if(!std::isfinite(c[i])) throw std::runtime_error("invalid position feedback");}
    std::cout<<"Selected motors ready; skipped IDs:"; for(int i=0;i<5;i++) if(!active[i]) std::cout<<" "<<(i+1); std::cout<<"; amplitude="<<amp<<" rad; id2 amplitude="<<amp_id2<<" rad; speed="<<speed<<" rad/s; id2 speed="<<speed_id2<<" rad/s; duration="<<duration<<" s\n";
    auto start=std::chrono::steady_clock::now(); int cycle = 0;
    while(!stop){double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count(); if(t>=duration)break; for(int i=0;i<5;i++) if(active[i]) { const double a = (i == 1) ? amp_id2 : amp; const double v = (i == 1) ? speed_id2 : speed; const double q=a*std::sin(2*M_PI*freq*t); m[i]->RobStrite_Motor_PosCSP_control((float)v,(float)(c[i]+q)); } if (monitor_id2_current && (++cycle % 5 == 0)) { m[1]->Get_RobStrite_Motor_parameter(0x701A); std::cout << "motor2 iqf: " << m[1]->drw.iqf.data << " A\n"; } std::this_thread::sleep_for(std::chrono::milliseconds(20));}
    if (hold_enabled && !stop) {
      std::cout << "Motion complete; motors remain enabled at their centers. Press Ctrl+C to disable.\n";
      while (!stop) {
        for (int i=0;i<5;i++) if(active[i])
          m[i]->RobStrite_Motor_PosCSP_control((float)((i == 1) ? speed_id2 : speed), (float)c[i]);
        if (monitor_id2_current && (++cycle % 5 == 0)) { m[1]->Get_RobStrite_Motor_parameter(0x701A); std::cout << "motor2 iqf: " << m[1]->drw.iqf.data << " A\n"; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  } catch(const std::exception&e){std::cerr<<"Stopping all motors: "<<e.what()<<"\n";}
  for(auto &x:m)if(x){try{x->Disenable_Motor(0);}catch(...) {}} std::cout<<"All motors stopped.\n"; return 0;
}
