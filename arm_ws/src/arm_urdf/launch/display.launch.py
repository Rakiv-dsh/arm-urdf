import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """机械臂 URDF 可视化 + 关节滑块。

    默认启动：robot_state_publisher + 官方 joint_state_publisher_gui（滑块）+ rviz2
    常用参数：
        rviz:=false            不启动 rviz（只发布 /joint_states 和 /tf）
        gui:=false             不启动滑块窗口
        fallback_slider:=true  用本包自写的 PyQt 滑块代替官方 GUI（官方包装不上时的后备）
    """
    share = get_package_share_directory("arm_urdf")
    urdf_path = os.path.join(share, "urdf", "arm_urdf.urdf")
    rviz_path = os.path.join(share, "rviz", "arm.rviz")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration("rviz")
    use_gui = LaunchConfiguration("gui")
    use_fallback = LaunchConfiguration("fallback_slider")

    # joint_state_publisher(_gui) 也需要 robot_description 才知道有哪些关节、限位多少
    jsp_params = [{"robot_description": robot_description, "rate": 30.0}]

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="启动 rviz2"),
            DeclareLaunchArgument(
                "gui", default_value="true", description="启动官方 joint_state_publisher_gui 滑块窗口"
            ),
            DeclareLaunchArgument(
                "fallback_slider",
                default_value="false",
                description="改用自写的 PyQt 滑块（官方 GUI 装不上时的后备）",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": False}
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=jsp_params,
                condition=IfCondition(use_gui),
            ),
            Node(
                package="arm_urdf",
                executable="joint_slider.py",
                name="joint_slider",
                output="screen",
                condition=IfCondition(use_fallback),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_path],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
