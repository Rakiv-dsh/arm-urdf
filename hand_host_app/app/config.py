"""用户偏好的轻量级持久化。

这个模块只负责保存上位机本身的运行偏好，**不**负责保存任何与硬件相关的常量
或手型定义。每只手的串口/波特率/关节/触觉/遥操作配置都写在它自己的
`app/resources/hands/<hand_id>/manifest.json` 里，由 `app.hands` 子包加载。

目前唯一的偏好项是 ``active_hand``：上一次用户在主窗口选中的手型 ID。
启动时主窗口读取它并切到对应手型；用户在侧边栏切换手型后再写回去。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


# config.json 与项目根目录平级，方便用户手工查看 / 备份。
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config.json"

# 没有任何手型时的占位 ID，与 app.hands.EMPTY_HAND_ID 同义；这里直接写字面量，
# 避免在加载 config 时反向 import app.hands。
DEFAULT_CONFIG: dict[str, str] = {
    "active_hand": "__empty__",
}


def _normalize_config(payload: Any) -> dict[str, str]:
    """把任意输入归一成"只含已知字段的合法 config 字典"。

    config.json 是用户可手改的明文 JSON，可能被改坏；这里宽容处理，遇到
    任何不识别的字段都丢掉，回落到 DEFAULT_CONFIG，保证主程序永远拿到
    一个结构稳定的字典。
    """
    config = dict(DEFAULT_CONFIG)
    if not isinstance(payload, dict):
        return config

    active_hand = payload.get("active_hand")
    if isinstance(active_hand, str) and active_hand.strip():
        config["active_hand"] = active_hand.strip()
    return config


def load_config() -> dict[str, str]:
    """读取用户偏好；文件不存在或损坏时返回默认配置。"""
    if not CONFIG_PATH.exists():
        return dict(DEFAULT_CONFIG)
    try:
        payload = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        # 损坏的 config.json 不应让上位机启动失败；用默认值继续，下次保存时会
        # 自动覆盖回合法格式。
        return dict(DEFAULT_CONFIG)
    return _normalize_config(payload)


def save_config(config: dict[str, Any]) -> None:
    """把用户偏好持久化到磁盘；只保留已知字段。"""
    normalized = _normalize_config(config)
    CONFIG_PATH.write_text(
        json.dumps(normalized, indent=4, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
