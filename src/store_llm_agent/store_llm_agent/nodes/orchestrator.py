import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from store_interfaces.action import ExecuteRobotTask
from store_interfaces.srv import ScanWorkspace


class OrchestratorNode(Node):
    """Integrates Mode FSM, RDFLib/SPARQL KG, GPT-4o FC, and robot task orchestration."""

    def __init__(self) -> None:
        super().__init__('orchestrator')
        self.mode_pub = self.create_publisher(String, '/store/mode/state', 10)
        self.ui_pub = self.create_publisher(String, '/store/ui_snapshot', 10)

        self.create_subscription(String, '/store/voice/transcript', self.on_transcript, 10)
        self.create_subscription(String, '/store/text_cmd', self.on_text_cmd, 10)
        self.create_subscription(String, '/store/vision/vest_detected', self.on_vest, 10)
        self.create_subscription(Bool, '/store/emergency_stop', self.on_stop, 10)

        self.scan_client = self.create_client(ScanWorkspace, 'scan_workspace')
        self.robot_client = ActionClient(self, ExecuteRobotTask, 'execute_robot_task')
        self.get_logger().info('orchestrator initialized (skeleton)')

    def on_transcript(self, msg: String) -> None:
        _ = msg

    def on_text_cmd(self, msg: String) -> None:
        _ = msg

    def on_vest(self, msg: String) -> None:
        _ = msg

    def on_stop(self, msg: Bool) -> None:
        _ = msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
