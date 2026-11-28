from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="0.0.0.0"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("stale_timeout_sec", default_value="6.0"),
        Node(
            package="store_web_ui",
            executable="web_ui_node",
            parameters=[{
                name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                for name, kind in (("host", str), ("port", int), ("stale_timeout_sec", float))
            }],
            output="screen",
        ),
    ])
