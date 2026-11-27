from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_remote(context):
    parameters = {"dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool)}
    for key, value_type in (("config_dir", str), ("velocity", float), ("acceleration", float)):
        value = LaunchConfiguration(key).perform(context)
        if value:
            parameters[key] = value_type(value)
    return [Node(package="store_robot_control", executable="remote_node",
                 parameters=[parameters], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("config_dir", default_value=""),
        DeclareLaunchArgument("velocity", default_value=""),
        DeclareLaunchArgument("acceleration", default_value=""),
        OpaqueFunction(function=_launch_remote),
    ])
