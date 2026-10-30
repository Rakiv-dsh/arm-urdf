import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """滑块 + 真机：让 joint_state_publisher_gui 的滑杆驱动五台 EL05。

    限位/电机号/方向/偏置全部来自 config/joint_limits.yaml（和 URDF 同一份数据源）。
    """
    share = get_package_share_directory("arm_urdf")
    urdf_path = os.path.join(share, "urdf", "arm_urdf.urdf")
    cfg_path = os.path.join(share, "config", "joint_limits.yaml")
    rviz_path = os.path.join(share, "rviz", "arm.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    joints = cfg["joints"]
    driven = [j for j in joints if j.get("motor")]
    motors = [j["motor"] for j in driven]

    servo_params = {
        "joint_names": [j["name"] for j in driven],
        "motor_ids": [int(m["can_id"]) for m in motors],
        "mech_offset": [float(m["offset"]) for m in motors],
        "joint_direction": [float(m["dir"]) for m in motors],
        "gear_ratio": [float(m["gear"]) for m in motors],
        "lower_limit": [float(j["lower"]) for j in driven],
        "upper_limit": [float(j["upper"]) for j in driven],
    }

    import launch_ros.parameter_descriptions as _pd  # noqa: F401
    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    use_servo = LaunchConfiguration("servo")
    can_iface = LaunchConfiguration("can_interface")
    max_speed = LaunchConfiguration("max_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true", description="滑块窗口"),
            DeclareLaunchArgument("servo", default_value="true",
                                  description="启动真机伺服（默认开，但还要 /arm/enable=true 才动）"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("max_speed", default_value="0.3",
                                  description="目标斜坡速度 rad/s，滑块猛拉也不会一步到位"),
            DeclareLaunchArgument("limit_cur", default_value="6.0",
                                  description="使能前把电机电流限幅写成这个值（A），0=不改"),
            DeclareLaunchArgument("watchdog_ms", default_value="1500.0",
                                  description="多久收不到 /joint_states 就失能；0=关闭"),
            DeclareLaunchArgument("active_joints", default_value="",
                                  description="只驱动这些关节，逗号分隔（空=全部），调方向时只填一个"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description, "rate": 30.0}],
                condition=IfCondition(use_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_path],
                condition=IfCondition(use_rviz),
            ),
            Node(
                package="rs_motor_ros2",
                executable="el05_slider_servo",
                name="el05_slider_servo",
                output="screen",
                parameters=[
                    servo_params,
                    {
                        "can_interface": can_iface,
                        "max_speed": max_speed,
                        "control_rate_hz": 50.0,
                        "speed_limit": 0.5,
                        "watchdog_ms": LaunchConfiguration("watchdog_ms"),
                        "limit_cur": LaunchConfiguration("limit_cur"),
                        "active_joints": LaunchConfiguration("active_joints"),
                    },
                ],
                condition=IfCondition(use_servo),
            ),
        ]
    )
