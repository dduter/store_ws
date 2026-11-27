import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Bool
from store_interfaces.action import ExecuteRobotTask
from store_interfaces.msg import NodeHealth


class MockRobotNode(Node):
    """Mock robot action server for hardware-free Day 2+ integration testing."""

    def __init__(self) -> None:
        super().__init__('mock_robot')
        self.health_pub = self.create_publisher(NodeHealth, '/store/node_health', 10)
        self.create_subscription(Bool, '/store/emergency_stop', self.on_stop, 10)
        self._action_server = ActionServer(
            self,
            ExecuteRobotTask,
            'execute_robot_task',
            self.execute_callback,
        )
        self.get_logger().info('mock_robot initialized (skeleton)')

    def on_stop(self, msg: Bool) -> None:
        _ = msg

    def execute_callback(self, goal_handle):
        goal_handle.succeed()
        result = ExecuteRobotTask.Result()
        result.success = True
        result.stopped_for_safety = False
        result.error_code = 'OK'
        result.final_pose = list(goal_handle.request.dest_pose)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
