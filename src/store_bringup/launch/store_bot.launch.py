from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_robot", default_value="true"),
        DeclareLaunchArgument("config_dir", default_value=""),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("step_delay_sec", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("store_bringup"), "launch", "mock_robot.launch.py",
            ])),
            condition=IfCondition(LaunchConfiguration("use_mock_robot")),
            launch_arguments={
                "config_dir": LaunchConfiguration("config_dir"),
                "step_delay_sec": LaunchConfiguration("step_delay_sec"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("store_bringup"), "launch", "executor.launch.py",
            ])),
            condition=UnlessCondition(LaunchConfiguration("use_mock_robot")),
            launch_arguments={
                "config_dir": LaunchConfiguration("config_dir"),
                "dry_run": LaunchConfiguration("dry_run"),
            }.items(),
        ),
        # 인식 노드와 오케스트레이터는 준비되면 이 목록에 추가합니다.
    ])
