# hand-host 上位机接入说明（arm_el05）

## 放在哪

上位机本体在 Windows：

    C:\Users\Raki\hand-host_arm\

**必须是纯英文路径**——MuJoCo 打不开中文路径（`C:\工作\...` 会直接抛 `Error opening file`）。

## 怎么启动

1. WSL 里起桥（真机）：

       cd ~/ros2_arm_ws/src/arm_urdf/tools
       python3 can_bridge.py --port 5599

   调试界面时用假数据：

       python3 can_bridge.py --port 5599 --dry-run

2. Windows 上起界面：

       cd C:\Users\Raki\hand-host_arm
       python main.py

3. 在「连接管理」页点 **连接**，状态变绿后可拖「关节控制」页的滑块。

## 包结构

    app/resources/hands/arm_el05/
      manifest.json              dexhand.hand.v1，5 个关节 / 电机 21~25 / 协议 bridge
      calibration.json           ±180° 限位（度）
      driver_adapter/driver.py   TCP 客户端，连 127.0.0.1:5599
      model/arm_el05.xml         MJCF（由 arm_real.urdf 转换，含 STL 网格）

## 对模板本身做的两处修改

1. `app/resources/hands/arm_el05/model/arm_el05.xml`：所有 `<geom>` 加 `group="1"`。
   模板的 3D 控件只看 group 1 的几何（`geomgroup[0]=0, geomgroup[1]=1`），
   放默认 group 0 会显示全黑。

2. `app/main_window.py`：加了一段全局 QSS 给 QLabel / QGroupBox / QComboBox 设深色文字。
   Windows 深色主题下，未显式设色的控件文字是白色，落在白底上完全看不见。

## 角度约定

界面上的角度是**关节角（度）**，和 `joint_limits.yaml` 的 offset 对齐：

    joint_deg = (mechPos_rad - offset) / dir

桥内部再换算回电机的机械角，所以界面上的 0° 就是我们的关节零位。

## 已验证

- manifest 通过上位机自带校验（0 错误）
- 连接页显示 M21~M25
- 关节控制页 5 条滑块，范围 ±180°，拖动时 3D 模型实时跟随
- 连接状态显示「arm_el05 已连接」

## 尚未验证

- 真机（非 dry-run）的使能/点动：还没在真机上走过一遍
- 「预设手势」「数据记录」页：还没接我们的关节名

## 界面裁剪（只保留机械臂相关）

侧边栏现在只剩 6 页：

    连接管理 / 关节控制 / 电机标定 / 数据记录 / 动作序列 / 机型管理

- 移除「触觉可视化」「仿真遥操作」（手部专用）
- 「预设手势」改名为「动作序列」，「手型管理」改名为「机型管理」
- 窗口标题改成「机械臂上位机控制系统」，侧栏标题「机械臂控制」
- 连接页在没有触觉的机型下自动隐藏「触觉通信」区块
- bridge 协议下不显示「无可用端口」，改为默认桥地址 127.0.0.1:5599（默认）

改动的文件（项目目录 docs/patches/ 有完整副本）：
main_window.py、widgets/connection_page.py、widgets/gesture_page.py、
widgets/hand_manager_page.py、widgets/calibration_page.py、widgets/tactile_hand_3d_widget.py
