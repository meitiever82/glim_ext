'''
Author: meitiever
Date: 2026-09-12 17:26:30
LastEditors: meitiever
LastEditTime: 2026-09-12 22:16:08
Description: content
'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("gnss_bringup"), "config", "gnss_bringup.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        # RTKLIB 未装时(见 README 缺口登记)用 enable_rtkrcv:=false 只起桥
        DeclareLaunchArgument("enable_rtkrcv", default_value="false"),
        # pos_writer 只依赖 gnss_msgs/RtkFix,不依赖 rtkrcv_node/RTKLIB 本身,
        # 默认随桥一起起来(与 enable_rtkrcv 相反,默认 true)——.pos 落盘是
        # 记录面常态开启的一部分,不需要现场先确认 RTKLIB 才能用。
        DeclareLaunchArgument("enable_pos_writer", default_value="true"),
        Node(package="gnss_bringup", executable="rtcm_bridge", name="rtcm_bridge",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
        Node(package="gnss_bringup", executable="rtkrcv_node", name="rtkrcv_node",
             parameters=[LaunchConfiguration("params_file")], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_rtkrcv"))),
        Node(package="gnss_bringup", executable="pos_writer", name="pos_writer",
             parameters=[LaunchConfiguration("params_file")], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_pos_writer"))),
    ])
