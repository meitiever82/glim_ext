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
        DeclareLaunchArgument("enable_rtkrcv", default_value="true"),
        Node(package="gnss_bringup", executable="rtcm_bridge", name="rtcm_bridge",
             parameters=[LaunchConfiguration("params_file")], output="screen"),
        Node(package="gnss_bringup", executable="rtkrcv_node", name="rtkrcv_node",
             parameters=[LaunchConfiguration("params_file")], output="screen",
             condition=IfCondition(LaunchConfiguration("enable_rtkrcv"))),
    ])
