"""手型（HandProfile）层。

整个上位机不绑定任何具体硬件，每只手都靠 ``app/resources/hands/<hand_id>/``
下的"手型包"（manifest.json + 适配器 .py + MuJoCo XML + 标定 JSON）来描述。

子模块分工：
    base       — HandProfile / FingerConfig / JointInfo 等数据类
    manifest   — 解析并校验 manifest.json，生成 HandProfile
    registry   — 已安装手型的全局注册表，记录"当前活动手型"
    adapter    — 动态加载手型包里的 driver.py（电机驱动适配器）
    runtime    — 根据手型 profile 创建对应的 MotorService 实例

应用其他模块（widgets / services）只通过这里暴露的工厂和注册表
访问手型信息，不应硬编码任何 hand_id。
"""

from app.hands.base import (
    NUM_JOINTS,
    FingerConfig,
    HandProfile,
    JointInfo,
)
from app.hands.registry import (
    ALL_JOINT_NAMES,
    FINGER_CONFIGS,
    JOINT_INDEX_TO_MOTOR_ID,
    MOTOR_ID_TO_JOINT_INDEX,
    clone_profile,
    delete_profile,
    EMPTY_HAND_ID,
    EMPTY_PROFILE,
    get_active_profile,
    get_profile,
    get_profiles,
    reload_profiles,
    set_active_profile,
)

__all__ = [
    "ALL_JOINT_NAMES",
    "FINGER_CONFIGS",
    "JOINT_INDEX_TO_MOTOR_ID",
    "MOTOR_ID_TO_JOINT_INDEX",
    "NUM_JOINTS",
    "FingerConfig",
    "HandProfile",
    "JointInfo",
    "clone_profile",
    "delete_profile",
    "EMPTY_HAND_ID",
    "EMPTY_PROFILE",
    "get_active_profile",
    "get_profile",
    "get_profiles",
    "reload_profiles",
    "set_active_profile",
]
