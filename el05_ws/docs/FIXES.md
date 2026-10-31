# 改动与已知坑

本仓库在上游基础上做了 4 处改动，都附了症状，便于日后回溯。

## 1. 故障字合成错误（会直接停机）— 已修

**文件**：`src/robstride_ros_sample/src/motor_cfg.cpp`

```cpp
// 原来
fault_value_ = (static_cast<uint32_t>(error_code) << 16) | (extra_data & 0xFFFF);
```

反馈帧的扩展 ID 是 `(通信类型<<24)|(故障码<<16)|(电机ID<<8)|(主机ID)`，
`extra_data` 取的是 `(id>>8)&0xFFFF`，低 8 位正好是**电机 ID**。于是只要电机 ID > 0，
`has_fault()` 就恒为真。

**症状**：`five_dof_arm_controller` 的安全检查里有 `motors_[i]->has_fault()`，
于是第一步控制就打印 `Safety stop on joint1`，电机永远不动。已实测复现。

```cpp
// 现在
fault_value_ = static_cast<uint32_t>(error_code);   // 只取 bit16~21
```

## 2. 位置反馈量纲的小偏差 — 未改（约 0.002 rad）

接收端按 `u16/32767-1` 解码，而真机编码是 `u16 = (x-min)/(max-min)*65535`。
满量程处差千分之几，实际影响 < 0.002 rad，未改动以免与上游行为分叉；虚拟电机按真机 65535 编码。

## 3. 死总线上连发会 ENOBUFS — 工具侧容错

总线上没有第二个节点 ACK 时，CAN 控制器会进 ERROR-PASSIVE，发送队列顶满，
`send()` 抛 `OSError: [Errno 105] No buffer space available`。
`tools/` 里的脚本对此做了重试容错——**这不是脚本 bug，而是总线没人应答的症状**。

## 4. 改 CAN ID 的「同 ID 多台」陷阱 — 工具侧加守卫

EL05 出厂 CAN ID 常见为 127。改 ID 帧 `0x07` 是**发给某个 motor_id 的**，
如果总线上并着多台同 ID 的电机，它们会**一起收到、一起改号**。
2026-10-29 实机踩到：4 台同挂一条总线、全是 127，一条「127→1」把它们全改成了 1 号；
之后单发一条指令会被 4 台同时执行。

**正确做法**：改号前用 `el05_set_id.py` 的节点计数确认该 ID 上只有一台；
多于一台先物理隔离（一次只给一台通电 / 只接一台 CAN）。
工具现在检测到同 ID 多台会直接拒绝，除非显式加 `--force`。

## 顺带记下的硬件事实（来自 EL05 / USB_CANHUB 手册）

- EL05：额定 48 VDC（15~60 V），额定扭矩 1.8 N·m / 峰值 6 N·m，额定相电流 2.6 Apk / 最大 11 Apk，减速比 9:1，编码器 14 bit 单圈绝对值。
- 电机接口：**XT30(2+2) 四芯**（2 芯电源 + 2 芯 CAN 在同一个插头里）。
- USB_CANHUB：Type-C 5 V 供电，每路 **2.54 mm 3P 端子（H / L / GND）**，每路有 **120 Ω 拨码开关**（ON = 接入），每路一对收发指示灯（绿收红发）。
- 私有协议两套可选协议：CANopen / MIT（MIT 用 11 位标准帧，切换后需重新上电）。
