"""力控服务 — 基于Q轴电流的逐电机堵转检测与力反馈抓握

原理:
  τ = K_t × I_q, 每个电机独立检测
  |I_q|(EMA滤波) > 阈值 → 堵转/过载 → 冻结合拢方向
  |I_q| 在 [50%阈值, 阈值] → 比例减速
  张开方向始终不限制

冻结恢复渐进:
  电机从冻结/减速状态恢复后，命令位置不会立即跳回远处目标，
  而是以 ramp_deg_per_tick 的速率渐进接近目标，避免瞬间大电流
  导致的反复振荡 (冻结↔恢复 循环)。

使用方式:
  fc = ForceControlService()
  adjusted = fc.apply(target_angles, telemetry)
"""

import time
from dataclasses import dataclass, field

from app.hands import get_active_profile

# 力控动作类型
ACTION_FREE = 0       # 自由跟随
ACTION_SLOWING = 1    # 比例减速
ACTION_FROZEN = 2     # 冻结


@dataclass
class ForceControlConfig:
    torque_scale: float = 1.0         # 力矩缩放系数 (乘以每关节 torque_limit)
    soft_ratio: float = 0.5           # 比例减速起点 = limit * soft_ratio
    ema_alpha: float = 0.3            # EMA 滤波系数 (越大越跟手)
    log_interval: float = 0.5         # 日志输出最小间隔 (秒)
    converge_tolerance: float = 0.5   # 收敛判定容差 (度)
    ramp_deg_per_tick: float = 5.0    # 冻结恢复后每tick最大步进 (度, 20Hz → 100°/s)
    min_force_delta: float = 2.0      # 距目标<此角度时跳过力控 (正常伺服电流，非堵转)


class ForceControlService:
    """逐电机力控服务 — 无UI依赖，可被任何页面调用"""

    def __init__(self, config: ForceControlConfig | None = None, profile=None):
        self.config = config or ForceControlConfig()
        self.profile = profile or get_active_profile()
        self._ema: dict[int, float] = {}          # motor_id → 电流EMA值
        self._last_action: dict[int, int] = {}    # motor_id → 上次动作
        self._ramp_active: set[int] = set()        # 需要渐进恢复的电机集合
        self._last_log_time: float = 0.0
        self._log_buffer: list[str] = []

    def reset(self):
        """重置滤波状态（启用力控或切换手势时调用）"""
        self._ema.clear()
        self._last_action.clear()
        self._ramp_active.clear()
        self._log_buffer.clear()

    def apply(
        self,
        target_angles: list[float],
        telemetry: dict[int, dict],
    ) -> list[float]:
        """对目标角度施加力控约束，返回调整后的角度列表

        内部从 telemetry 提取电机实际位置，确保 delta 计算准确。
        曾被冻结/减速的电机恢复后会渐进逼近目标，防止振荡。

        Args:
            target_angles: 期望的关节角度列表
            telemetry: {motor_id: {current, position, ...}} 电机遥测

        Returns:
            调整后的关节角度列表
        """
        cfg = self.config
        result = list(target_angles)
        now = time.monotonic()
        should_log = (now - self._last_log_time) >= cfg.log_interval

        jidx = 0
        for finger in self.profile.finger_configs:
            finger_joints = finger.joints
            for j, joint in enumerate(finger_joints):
                if jidx >= len(result):
                    break
                mid = self.profile.joint_index_to_motor_id.get(jidx)
                if mid is None:
                    jidx += 1
                    continue

                # 每关节独立力矩阈值 = 关节默认值 × UI缩放系数
                joint_limit = joint.torque_limit * cfg.torque_scale
                soft_start = joint_limit * cfg.soft_ratio

                t = telemetry.get(mid, {})
                if not t.get('online', False):
                    jidx += 1
                    continue  # 电机不在线，跳过

                # 从遥测读电机实际位置（不是 hand_state 的命令位置）
                actual_pos = t.get('position', result[jidx])

                # 读取Q轴电流，EMA滤波
                raw_cur = abs(t.get('current', 0.0))
                prev_ema = self._ema.get(mid, 0.0)
                cur_ema = cfg.ema_alpha * raw_cur + (1.0 - cfg.ema_alpha) * prev_ema
                self._ema[mid] = cur_ema

                if cur_ema < soft_start:
                    self._set_action(mid, ACTION_FREE)

                    # 曾被力控干预的电机：渐进恢复，不直接跳到目标
                    if mid in self._ramp_active:
                        target = result[jidx]
                        remaining = target - actual_pos
                        if abs(remaining) <= cfg.converge_tolerance:
                            # 已接近目标，结束渐进
                            self._ramp_active.discard(mid)
                        else:
                            # 限制每tick步进
                            max_step = cfg.ramp_deg_per_tick
                            if abs(remaining) > max_step:
                                sign = 1.0 if remaining > 0 else -1.0
                                result[jidx] = actual_pos + sign * max_step
                            # else: remaining <= max_step, 直接用 target (result 已是 target)
                    jidx += 1
                    continue

                target = result[jidx]
                delta = target - actual_pos

                # 距目标极近时跳过力控：此时高电流是正常伺服行为，非堵转
                if abs(delta) < cfg.min_force_delta:
                    self._ramp_active.discard(mid)
                    self._set_action(mid, ACTION_FREE)
                    jidx += 1
                    continue

                # 方向判断：远离 default = 合拢
                default = joint.default_angle
                closing_sign = 1.0 if actual_pos >= default else -1.0
                is_closing = (delta * closing_sign) > 0

                if not is_closing:
                    self._set_action(mid, ACTION_FREE)
                    jidx += 1
                    continue

                # 标记此电机需要渐进恢复（冻结/减速后不会瞬间跳回目标）
                self._ramp_active.add(mid)

                # 合拢方向力控
                if cur_ema >= joint_limit:
                    result[jidx] = actual_pos
                    action = ACTION_FROZEN
                    if should_log:
                        self._log_buffer.append(
                            f"  M{mid}({joint.display_name}): "
                            f"冻结 I={cur_ema:.0f}mA≥{joint_limit:.0f}mA "
                            f"actual={actual_pos:.1f}°"
                        )
                else:
                    ratio = (cur_ema - soft_start) / max(
                        joint_limit - soft_start, 1e-6)
                    desired = actual_pos + delta * (1.0 - ratio)
                    # 限制减速状态下的步进，防止大delta引起电流尖峰
                    max_step = cfg.ramp_deg_per_tick
                    cmd_delta = desired - actual_pos
                    if abs(cmd_delta) > max_step:
                        sign = 1.0 if cmd_delta > 0 else -1.0
                        result[jidx] = actual_pos + sign * max_step
                    else:
                        result[jidx] = desired
                    action = ACTION_SLOWING
                    if should_log:
                        self._log_buffer.append(
                            f"  M{mid}({joint.display_name}): "
                            f"减速{ratio*100:.0f}% I={cur_ema:.0f}mA "
                            f"actual={actual_pos:.1f}°→cmd={result[jidx]:.1f}°"
                        )

                self._set_action(mid, action)
                jidx += 1

        # 输出日志
        if should_log and self._log_buffer:
            self._last_log_time = now
            print(f"[力控] {time.strftime('%H:%M:%S')} 力反馈触发:")
            for line in self._log_buffer:
                print(line)
            self._log_buffer.clear()
        elif should_log:
            self._log_buffer.clear()

        return result

    def _set_action(self, mid: int, action: int):
        """记录动作并在状态变化时输出日志"""
        prev = self._last_action.get(mid, ACTION_FREE)
        self._last_action[mid] = action
        if action != prev:
            if action == ACTION_FROZEN:
                cur = self._ema.get(mid, 0.0)
                print(f"[力控] M{mid}: ⛔ 过载冻结 (I={cur:.0f}mA)")
            elif action == ACTION_SLOWING and prev == ACTION_FREE:
                cur = self._ema.get(mid, 0.0)
                print(f"[力控] M{mid}: ⚠ 开始减速 (I={cur:.0f}mA)")
            elif action == ACTION_FREE and prev != ACTION_FREE:
                print(f"[力控] M{mid}: ✓ 恢复自由 (渐进归位中)")

    def get_motor_ema(self, motor_id: int) -> float:
        """获取某电机当前EMA电流值(mA)，用于UI显示"""
        return self._ema.get(motor_id, 0.0)

    def get_active_count(self) -> int:
        """当前有多少电机处于减速或冻结状态"""
        return sum(1 for a in self._last_action.values() if a != ACTION_FREE)

    def get_ramping_count(self) -> int:
        """当前有多少电机在渐进恢复中"""
        return len(self._ramp_active)

    def is_converged(self, target_angles: list[float], telemetry: dict[int, dict]) -> bool:
        """检查所有电机是否已到达目标位置(在容差内)且无力控干预/渐进"""
        if self.get_active_count() > 0:
            return False
        if self._ramp_active:
            return False  # 仍有电机在渐进恢复中
        tol = self.config.converge_tolerance
        for jidx in range(min(self.profile.num_joints, len(target_angles))):
            mid = self.profile.joint_index_to_motor_id.get(jidx)
            if mid is None:
                continue
            t = telemetry.get(mid, {})
            if not t.get('online', False):
                continue
            actual = t.get('position', 0.0)
            if abs(actual - target_angles[jidx]) > tol:
                return False
        return True
