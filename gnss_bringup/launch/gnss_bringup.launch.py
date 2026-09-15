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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("gnss_bringup"), "config", "gnss_bringup.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        # RTKLIB 未装时(安装方法见 README「安装 RTKLIB-EX 2.5.1」,已验证/未验证的
        # 覆盖范围见「已验证 / 未验证项」)用 enable_rtkrcv:=false 只起桥
        DeclareLaunchArgument("enable_rtkrcv", default_value="false"),
        # pos_writer 只依赖 gnss_msgs/RtkFix,不依赖 rtkrcv_node/RTKLIB 本身,
        # 默认随桥一起起来(与 enable_rtkrcv 相反,默认 true)——.pos 落盘是
        # 记录面常态开启的一部分,不需要现场先确认 RTKLIB 才能用。
        DeclareLaunchArgument("enable_pos_writer", default_value="true"),
        # 诊断与清理默认随桥一起起来(与 pos_writer 相同):events.log / base.pos 与磁盘清理
        # 是记录面常态开启的一部分。rtkrcv 没启用时诊断把 no_solution 记成 info,不开事件。
        DeclareLaunchArgument("enable_diag", default_value="true"),
        DeclareLaunchArgument("enable_cleanup", default_value="true"),
        # 录包根目录:与 record_gnss.sh 同一套默认(先看 GNSS_BAG_ROOT,再退回 $HOME/gnss_bags)
        DeclareLaunchArgument(
            "bag_root",
            default_value=EnvironmentVariable(
                "GNSS_BAG_ROOT", default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "gnss_bags"]))),
        Node(package="gnss_bringup", executable="rtcm_bridge", name="rtcm_bridge",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
        Node(package="gnss_bringup", executable="rtkrcv_node", name="rtkrcv_node",
             parameters=[LaunchConfiguration("params_file")], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_rtkrcv"))),
        Node(package="gnss_bringup", executable="pos_writer", name="pos_writer",
             parameters=[LaunchConfiguration("params_file")], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_pos_writer"))),
        # solver_enabled 跟随 enable_rtkrcv,不在 yaml 里重复写一份(两处容易改漏)
        Node(package="gnss_bringup", executable="gnss_diag_node", name="gnss_diag",
             parameters=[LaunchConfiguration("params_file"),
                         {"solver_enabled": ParameterValue(LaunchConfiguration("enable_rtkrcv"), value_type=bool)}],
             output="screen",
             condition=IfCondition(LaunchConfiguration("enable_diag"))),
        Node(package="gnss_bringup", executable="gnss_cleanup_node", name="gnss_cleanup",
             parameters=[LaunchConfiguration("params_file"), {"bag_root": LaunchConfiguration("bag_root")}],
             output="screen",
             condition=IfCondition(LaunchConfiguration("enable_cleanup"))),
    ])
