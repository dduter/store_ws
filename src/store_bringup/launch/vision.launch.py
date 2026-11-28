from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=''),
        DeclareLaunchArgument('vlm_enabled', default_value='true'),
        DeclareLaunchArgument('use_live_robot_pose', default_value='true'),
        DeclareLaunchArgument('enable_webcam', default_value='true'),
        DeclareLaunchArgument('enable_realsense_mock', default_value='false'),
        Node(
            package='store_vision', executable='perception_node', output='screen',
            parameters=[{
                'config_dir': ParameterValue(LaunchConfiguration('config_dir'), value_type=str),
                'vlm_enabled': ParameterValue(LaunchConfiguration('vlm_enabled'), value_type=bool),
                'use_live_robot_pose': ParameterValue(
                    LaunchConfiguration('use_live_robot_pose'), value_type=bool,
                ),
            }],
        ),
        Node(
            package='store_vision', executable='webcam_node', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_webcam')),
        ),
        Node(
            package='store_vision', executable='realsense_node', output='screen',
            parameters=[{'mock_mode': True}],
            condition=IfCondition(LaunchConfiguration('enable_realsense_mock')),
        ),
    ])
