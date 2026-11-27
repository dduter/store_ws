from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_executor(context):
    parameters = {
        key: ParameterValue(LaunchConfiguration(key), value_type=bool)
        for key in ("dry_run", "check_grip_detection", "apply_grasp_depth_offset")
    }
    for key, value_type in (
        ("config_dir", str), ("velocity", float), ("acceleration", float), ("step_delay_sec", float),
    ):
        value = LaunchConfiguration(key).perform(context)
        if value:
            parameters[key] = value_type(value)
    return [Node(package="store_robot_control", executable="executor_node",
                 parameters=[parameters], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("config_dir", default_value=""),
        DeclareLaunchArgument("velocity", default_value=""),
        DeclareLaunchArgument("acceleration", default_value=""),
        DeclareLaunchArgument("check_grip_detection", default_value="true"),
        DeclareLaunchArgument("apply_grasp_depth_offset", default_value="true"),
        DeclareLaunchArgument("step_delay_sec", default_value=""),
        OpaqueFunction(function=_launch_executor),
    ])
