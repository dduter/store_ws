from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mock_robot_arg = DeclareLaunchArgument(
        'mock_robot',
        default_value='true',
        description='Use mock_robot instead of real M0609/RG2 executor_node',
    )
    mock_robot = LaunchConfiguration('mock_robot')

    return LaunchDescription([
        mock_robot_arg,
        Node(package='store_vision', executable='realsense_node', output='screen'),
        Node(package='store_vision', executable='webcam_node', output='screen'),
        Node(package='store_vision', executable='perception_node', output='screen'),
        Node(package='store_llm_agent', executable='voice_node', output='screen'),
        Node(package='store_llm_agent', executable='orchestrator', output='screen'),
        Node(
            package='store_robot_control',
            executable='mock_robot',
            output='screen',
            condition=IfCondition(mock_robot),
        ),
        Node(
            package='store_robot_control',
            executable='executor_node',
            output='screen',
            condition=UnlessCondition(mock_robot),
        ),
        Node(package='store_web_ui', executable='web_ui_node', output='screen'),
    ])
