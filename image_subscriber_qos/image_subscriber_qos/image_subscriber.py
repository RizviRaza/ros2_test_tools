import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos_event import QoSPublisherEventType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        # Declare and get parameters for both topics
        self.declare_parameter('topic1', '/camera/image1/compressed')
        self.declare_parameter('topic2', '/camera/image2/compressed')
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_history', 'keep_all')
        self.declare_parameter('qos_depth', 30)

        topic1 = self.get_parameter('topic1').get_parameter_value().string_value
        topic2 = self.get_parameter('topic2').get_parameter_value().string_value
        reliability = self.get_parameter('qos_reliability').get_parameter_value().string_value
        history = self.get_parameter('qos_history').get_parameter_value().string_value
        depth = self.get_parameter('qos_depth').get_parameter_value().integer_value

        reliability_policy = ReliabilityPolicy.RELIABLE if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT
        history_policy = HistoryPolicy.KEEP_LAST if history == 'keep_last' else HistoryPolicy.KEEP_ALL

        qos_profile = QoSProfile(
            reliability=reliability_policy,
            history=history_policy,
            depth=depth
        )

        # First subscription
        self.subscription1 = self.create_subscription(
            CompressedImage,
            topic1,
            self.listener_callback1,
            qos_profile,
            event_callbacks=SubscriptionEventCallbacks(
                message_lost=self.message_lost_callback1
            )
        )

        self.subscription2 = self.create_subscription(
            CompressedImage,
            topic2,
            self.listener_callback2,
            qos_profile,
            event_callbacks=SubscriptionEventCallbacks(
                message_lost=self.message_lost_callback2
            )
        )

        self.get_logger().info(f'Subscribed to {topic1} and {topic2} with QoS (reliability={reliability}, history={history}, depth={depth})')


    def message_lost_callback1(self, event):
        self.get_logger().warn(f"[Topic 1] Message lost! Total lost: {event.total_count}")

    def message_lost_callback2(self, event):
        self.get_logger().warn(f"[Topic 2] Message lost! Total lost: {event.total_count}")

    def listener_callback1(self, msg: CompressedImage):
        header = msg.header
        self.get_logger().info(
            f"[Topic 1] Header: stamp={header.stamp.sec}.{header.stamp.nanosec}, frame_id='{header.frame_id}' | Image size: {len(msg.data)} bytes"
        )

    def listener_callback2(self, msg: CompressedImage):
        header = msg.header
        self.get_logger().info(
            f"[Topic 2] Header: stamp={header.stamp.sec}.{header.stamp.nanosec}, frame_id='{header.frame_id}' | Image size: {len(msg.data)} bytes"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
