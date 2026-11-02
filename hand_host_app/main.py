"""灵巧手上位机控制系统 — 程序入口。

整个上位机是一个"通用空壳"：自身不绑定任何具体硬件，只负责
   1. 提供 PySide6 桌面 UI 与导航；
   2. 在启动时按用户上次选择，从 app/resources/hands/<hand_id>/ 加载手型包；
   3. 通过统一的适配器接口（电机、触觉、遥操作）和具体硬件交互。

本文件只做三件事：
   * 在 import PySide6 之前完成 Qt 环境变量与 warning 抑制（顺序敏感）；
   * 创建 QApplication 并设置全局样式；
   * 实例化 MainWindow 并进入 Qt 事件循环。
"""

import sys

# 必须在 PySide6 任何模块被 import 之前调用：
# configure_qt_environment 设置 QT_LOGGING_RULES，让 Qt 启动时不再因为字体探测
# 失败而往控制台刷错误；quiet_common_warnings 抑制 matplotlib 字体回退警告。
# 顺序如果颠倒，Qt 可能已经把日志规则缓存住，环境变量再设也没用。
from app.platform_compat import (
    configure_qt_appearance,
    configure_qt_environment,
    quiet_common_warnings,
)

configure_qt_environment()
quiet_common_warnings()

from PySide6.QtWidgets import QApplication

from app.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("灵巧手上位机")
    app.setOrganizationName("DexterousHand")
    configure_qt_appearance(app)

    # 全局浅色主题：导航按钮的 :checked 状态、tooltip 风格、滚动条都在这里集中定义，
    # 各页面里只需要负责自己的局部样式，不再重复写整套配色。
    app.setStyleSheet("""
        QMainWindow {
            background-color: white;
        }
        QWidget {
            color: #222222;
        }
        QGroupBox {
            font-weight: bold;
        }
        QToolTip {
            background-color: #333;
            color: white;
            border: none;
            padding: 4px 8px;
            border-radius: 4px;
        }
        QScrollBar:vertical {
            width: 8px;
            background: transparent;
        }
        QScrollBar::handle:vertical {
            background: #ccc;
            border-radius: 4px;
            min-height: 30px;
        }
        QScrollBar::handle:vertical:hover {
            background: #aaa;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0px;
        }
    """)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
