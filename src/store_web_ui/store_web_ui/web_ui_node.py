import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from store_interfaces.msg import NodeHealth


class WebUiNode(Node):
    """Monitoring Web UI bridge node (live view, KG, STOP, text fallback, heartbeat)."""

    def __init__(self) -> None:
        super().__init__('web_ui_node')
        self.text_cmd_pub = self.create_publisher(String, '/store/text_cmd', 10)
        self.stop_pub = self.create_publisher(Bool, '/store/emergency_stop', 10)

        self.create_subscription(String, '/store/ui_snapshot', self.on_snapshot, 10)
        self.create_subscription(NodeHealth, '/store/node_health', self.on_health, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.on_color, 10)
        self.create_subscription(Image, '/webcam/image_raw', self.on_webcam, 10)
        self.get_logger().info('web_ui_node initialized (skeleton)')

    def on_snapshot(self, msg: String) -> None:
        _ = msg

    def on_health(self, msg: NodeHealth) -> None:
        _ = msg

    def on_color(self, msg: Image) -> None:
        _ = msg

    def on_webcam(self, msg: Image) -> None:
        _ = msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebUiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
