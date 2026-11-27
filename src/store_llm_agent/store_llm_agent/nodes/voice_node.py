import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceNode(Node):
    """Microphone wakeword/PTT + Whisper STT publisher."""

    def __init__(self) -> None:
        super().__init__('voice_node')
        self.transcript_pub = self.create_publisher(String, '/store/voice/transcript', 10)
        self.get_logger().info('voice_node initialized (skeleton)')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
