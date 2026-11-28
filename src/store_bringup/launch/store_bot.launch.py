from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common = ("config_dir", "dry_run", "velocity", "acceleration")
    executor_options = ("step_delay_sec", "check_grip_detection", "apply_grasp_depth_offset")

    def include(launch_file, arguments, condition, **mapped_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("store_bringup"), "launch", launch_file,
            ])),
            condition=condition,
            launch_arguments={
                **{name: LaunchConfiguration(name) for name in arguments},
                **mapped_arguments,
            }.items(),
        )

    mock = LaunchConfiguration("use_mock_robot")
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_robot", default_value="true"),
        DeclareLaunchArgument("config_dir", default_value=""),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("step_delay_sec", default_value=""),
        DeclareLaunchArgument("enable_remote", default_value="false"),
        DeclareLaunchArgument("velocity", default_value=""),
        DeclareLaunchArgument("acceleration", default_value=""),
        DeclareLaunchArgument("check_grip_detection", default_value="true"),
        DeclareLaunchArgument("apply_grasp_depth_offset", default_value="true"),
        DeclareLaunchArgument("enable_vision", default_value="false"),
        DeclareLaunchArgument("enable_webcam", default_value="true"),
        DeclareLaunchArgument("enable_realsense_mock", default_value="false"),
        include(
            "vision.launch.py", ("config_dir", "enable_webcam", "enable_realsense_mock"),
            IfCondition(LaunchConfiguration("enable_vision")),
        ),
        DeclareLaunchArgument("enable_web_ui", default_value="false"),
        DeclareLaunchArgument("web_ui_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_ui_port", default_value="8080"),
        include(
            "web_ui.launch.py", (), IfCondition(LaunchConfiguration("enable_web_ui")),
            host=LaunchConfiguration("web_ui_host"), port=LaunchConfiguration("web_ui_port"),
        ),
        include("mock_robot.launch.py", ("config_dir", "step_delay_sec"), IfCondition(mock)),
        include("executor.launch.py", (*common, *executor_options), UnlessCondition(mock)),
        include("remote.launch.py", common, IfCondition(LaunchConfiguration("enable_remote"))),
    ])
