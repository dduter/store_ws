from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_robot(context):
    config_dir = LaunchConfiguration("config_dir").perform(context)
    step_delay = LaunchConfiguration("step_delay_sec").perform(context)
    parameters = {}
    if config_dir:
        parameters["config_dir"] = config_dir
    if step_delay:
        parameters["step_delay_sec"] = float(step_delay)
    return [Node(package="store_robot_control", executable="mock_robot",
                 parameters=[parameters], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_dir", default_value="", description="빈 값이면 설정 경로를 자동 탐색합니다."),
        DeclareLaunchArgument("step_delay_sec", default_value="", description="빈 값이면 YAML의 단계 간격을 사용합니다."),
        OpaqueFunction(function=_launch_robot),
    ])
