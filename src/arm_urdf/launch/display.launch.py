import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """在 launch 运行时读取 URDF，这样 urdf_file 参数才能生效。"""
    share = get_package_share_directory("arm_urdf")
    urdf_file = context.launch_configurations.get("urdf_file", "arm_urdf.urdf")
    urdf_path = os.path.join(share, "urdf", urdf_file)
    rviz_path = os.path.join(share, "rviz", "arm.rviz")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            parameters=[{"robot_description": robot_description, "rate": 30.0}],
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="arm_urdf",
            executable="joint_slider.py",
            name="joint_slider",
            output="screen",
            condition=IfCondition(LaunchConfiguration("fallback_slider")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    """只起可视化（模型 + 滑块 + RViz），不碰电机。

    urdf_file 决定用哪份模型：
      arm_urdf.urdf    —— 真实限位（和控制器一致）
      arm_display.urdf —— 限位放宽到 ±180°，专门用来拖模型对姿态
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="启动 rviz2"),
            DeclareLaunchArgument("gui", default_value="true", description="启动滑块窗口"),
            DeclareLaunchArgument("fallback_slider", default_value="false",
                                  description="用自写的 PyQt 滑块"),
            DeclareLaunchArgument("urdf_file", default_value="arm_urdf.urdf",
                                  description="arm_urdf.urdf（真实限位）/ arm_display.urdf（放宽，只调模型）"),
            OpaqueFunction(function=launch_setup),
        ]
    )
