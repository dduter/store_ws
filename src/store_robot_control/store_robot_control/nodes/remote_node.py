import rclpy
from rclpy.node import Node
from store_interfaces.srv import VerifyRemoteKey


class RemoteNode(Node):
    """(Post-MVP) Remote teleop receiver with Watchdog."""

    def __init__(self) -> None:
        super().__init__('remote_node')
        self.verify_srv = self.create_service(VerifyRemoteKey, 'verify_remote_key', self.handle_verify)
        self.get_logger().info('remote_node initialized (skeleton)')

    def handle_verify(self, request: VerifyRemoteKey.Request, response: VerifyRemoteKey.Response):
        _ = request
        response.granted = False
        response.reason = 'skeleton'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
