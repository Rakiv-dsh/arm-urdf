import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("arm_real")
    urdf_file = context.launch_configurations.get("urdf_file", "arm_real.urdf")
    urdf_path = os.path.join(share, "urdf", urdf_file)
    rviz_path = os.path.join(share, "rviz", "arm_real.rviz")
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description, "use_sim_time": False}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             name="joint_state_publisher_gui", output="screen",
             parameters=[{"robot_description": robot_description, "rate": 30.0}],
             condition=IfCondition(LaunchConfiguration("gui"))),
        # 自写滑块：界面上直接显示角度（°），用来对姿态更方便
        Node(package="arm_urdf", executable="joint_slider.py", name="joint_slider",
             output="screen", condition=IfCondition(LaunchConfiguration("fallback_slider"))),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz_path],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("urdf_file", default_value="arm_real.urdf"),
        DeclareLaunchArgument("fallback_slider", default_value="false",
                              description="用显示角度的自写滑块替代 joint_state_publisher_gui"),
        OpaqueFunction(function=launch_setup),
    ])
