'''
Author: meitiever
Date: 2026-09-12 17:26:30
LastEditors: meitiever
LastEditTime: 2026-09-12 22:16:08
Description: content
'''
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def resolve_bag_root(launch_arg, env):
    """录包根目录的解析顺序:launch 参数 bag_root 非空 → $GNSS_BAG_ROOT 非空 → $HOME 非空时 $HOME/gnss_bags → ""。

    与 record_gnss.sh 的 ${GNSS_BAG_ROOT:-$HOME/gnss_bags} 一致(:- 对空值同样回退)。
    HOME 缺失(systemd 系统单元不带 User= 时常见)不能让整个 launch 失败,返回 "" 由调用方处理。
    """
    if launch_arg:
        return launch_arg
    if env.get("GNSS_BAG_ROOT"):
        return env["GNSS_BAG_ROOT"]
    if env.get("HOME"):
        return os.path.join(env["HOME"], "gnss_bags")
    return ""


def cleanup_node(context):
    # 在 launch 时才解析 bag_root:Humble 的 EnvironmentVariable 没有默认值时会立即求值失败,
    # 放在 DeclareLaunchArgument 默认值里即使 enable_cleanup:=false 也会拖垮整个 bringup
    if not IfCondition(LaunchConfiguration("enable_cleanup")).evaluate(context):
        return []
    bag_root = resolve_bag_root(LaunchConfiguration("bag_root").perform(context), os.environ)
    actions = []
    if not bag_root:
        actions.append(LogInfo(msg="[WARN] gnss_cleanup: bag_root 未能确定(bag_root 参数、GNSS_BAG_ROOT、HOME "
                                   "都为空),本次只清理 pos_root;需要清理录包时传 bag_root:=<目录>"))
    actions.append(
        Node(package="gnss_bringup", executable="gnss_cleanup_node", name="gnss_cleanup",
             # value_type=str:空串若按 yaml 推断会变成 null
             parameters=[LaunchConfiguration("params_file"),
                         {"bag_root": ParameterValue(bag_root, value_type=str)}],
             output="screen"))
    return actions


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
        # 录包根目录:留空时在 launch 时按 GNSS_BAG_ROOT → $HOME/gnss_bags 解析,与 record_gnss.sh 一致
        # (见 resolve_bag_root)
        DeclareLaunchArgument("bag_root", default_value=""),
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
        OpaqueFunction(function=cleanup_node),
    ])
