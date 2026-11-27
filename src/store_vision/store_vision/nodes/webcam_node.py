import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class WebcamNode(Node):
    """Laptop webcam publisher for employee vest detection / remote teleop."""

    def __init__(self) -> None:
        super().__init__('webcam_node')
        self.image_pub = self.create_publisher(Image, '/webcam/image_raw', 10)
        self.create_subscription(String, '/store/mode/state', self.on_mode_state, 10)
        self.get_logger().info('webcam_node initialized (skeleton)')

    def on_mode_state(self, msg: String) -> None:
        _ = msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
