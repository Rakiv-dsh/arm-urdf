"""跨平台运行时兼容补丁。

这里集中处理"在不同平台/不同发行版上跑同一份代码会遇到的小坑"：
   - Qt 字体探测在某些 Linux 上会刷错误日志；
   - matplotlib 找不到中文字体时会刷一堆 Glyph 缺失警告；
   - WSL 上 MuJoCo offscreen 渲染分辨率太高会卡，需要缩小一下；
   - matplotlib 在 Linux 上要手动指定中文字体，否则坐标轴标签是方框。
"""

from __future__ import annotations

import logging
import os
import warnings
from functools import lru_cache

from PySide6.QtGui import QFont, QFontDatabase, QPalette, QColor
from PySide6.QtWidgets import QApplication


def configure_qt_environment() -> None:
    """在 QApplication 创建之前设置 Qt 环境变量。

    必须在 import PySide6 之前调用，否则 Qt 已经把日志规则缓存住，再设也没用。
    """
    # 关闭 qt.qpa.fonts 类别日志，避免启动时刷一屏字体探测错误。
    os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts=false")


def quiet_common_warnings() -> None:
    """抑制不会影响 UI 的噪声警告（matplotlib 字体回退、Axes3D 等）。"""
    warnings.filterwarnings(
        "ignore",
        message=r"Unable to import Axes3D.*",
        category=UserWarning,
    )
    warnings.filterwarnings(
        "ignore",
        message=r"Glyph .* missing from font\(s\).*",
        category=UserWarning,
    )
    logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)


def configure_qt_appearance(app: QApplication) -> None:
    """统一 Qt 的基础字体和浅色调色板，减少不同机器上的显示差异。"""
    preferred_families = (
        "Microsoft YaHei",
        "SimHei",
        "PingFang SC",
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "Arial",
    )
    installed = set(QFontDatabase.families())

    font = QFont()
    for family in preferred_families:
        if family in installed:
            font.setFamily(family)
            break
    if font.family() == app.font().family():
        font = app.font()
    if font.pointSize() <= 0:
        font.setPointSize(10)
    elif font.pointSize() < 10:
        font.setPointSize(10)
    app.setFont(font)

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.WindowText, QColor("#222222"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#f5f5f5"))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor("#333333"))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#222222"))
    palette.setColor(QPalette.ColorRole.Button, QColor("#f2f2f2"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor("#222222"))
    palette.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#0078d4"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    app.setPalette(palette)


@lru_cache(maxsize=1)
def is_wsl() -> bool:
    """是否运行在 WSL（Windows Subsystem for Linux）里。

    WSL 下 OpenGL 走软件渲染，需要降低 MuJoCo 渲染分辨率才不卡。
    """
    try:
        with open("/proc/sys/kernel/osrelease", "r", encoding="utf-8") as f:
            release = f.read().lower()
    except OSError:
        return False
    return "microsoft" in release or "wsl" in release


@lru_cache(maxsize=1)
def render_scale() -> float:
    """返回 MuJoCo offscreen 渲染缩放比例。

    可通过环境变量 HAND_HOST_RENDER_SCALE 自定义；< 1 表示先小尺寸渲染再让
    Qt 拉伸到控件大小，可以换帧率。WSL 默认 0.65，其他平台默认 1.0。
    """
    value = os.environ.get("HAND_HOST_RENDER_SCALE", "").strip()
    if value:
        try:
            return float(min(max(float(value), 0.35), 1.0))
        except ValueError:
            pass
    return 0.65 if is_wsl() else 1.0


def scaled_render_size(width: int, height: int, minimum: int = 160) -> tuple[int, int]:
    """根据 render_scale() 把目标显示尺寸缩到 offscreen 渲染尺寸。"""
    scale = render_scale()
    return (
        int(max(minimum, round(float(width) * scale))),
        int(max(minimum, round(float(height) * scale))),
    )


@lru_cache(maxsize=1)
def configure_matplotlib_fonts() -> bool:
    """为 matplotlib 选一款支持中文的字体。

    返回值:
        True  — 系统装了 CJK 字体，标签可以正常显示中文；
        False — 没找到，回落到 DejaVu Sans，调用方可以决定是否退化成英文标签。
    """
    quiet_common_warnings()

    import matplotlib
    from matplotlib import font_manager

    # 按 Windows / macOS / Linux 的常见 CJK 字体顺序探测。命中第一个就用它。
    cjk_candidates = (
        # Windows
        "Microsoft YaHei",
        "SimHei",
        "SimSun",
        "KaiTi",
        # macOS
        "PingFang SC",
        "Heiti SC",
        "Songti SC",
        # Linux distributions
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Noto Serif CJK SC",
        "Source Han Sans SC",
        "Source Han Serif SC",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
        "AR PL UMing CN",
        "Droid Sans Fallback",
    )
    installed = {font.name for font in font_manager.fontManager.ttflist}
    selected = next((name for name in cjk_candidates if name in installed), None)

    matplotlib.rcParams["axes.unicode_minus"] = False
    if selected:
        matplotlib.rcParams["font.family"] = ["sans-serif"]
        matplotlib.rcParams["font.sans-serif"] = [selected, "DejaVu Sans"]
        return True

    matplotlib.rcParams["font.family"] = ["DejaVu Sans"]
    matplotlib.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    return False
