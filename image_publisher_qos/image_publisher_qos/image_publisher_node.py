import rclpy

from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge

import cv2

from rclpy.qos import QoSProfile, ReliabilityPolicy
 
class ImagePublisherNode(Node):

    def __init__(self):

        super().__init__('image_publisher_node')
 
        # Declare parameters for the node

        self.declare_parameter('topic_name', 'image_topic/compressed')

        self.declare_parameter('image_path', 'image.png')

        self.declare_parameter('publish_rate', 1.0)
 
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        image_path = self.get_parameter('image_path').get_parameter_value().string_value

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
 
        # QoS settings for reliability

        qos_profile = QoSProfile(depth=50)

        qos_profile.reliability = ReliabilityPolicy.RELIABLE
 
        # Publisher setup for CompressedImage

        self.publisher_ = self.create_publisher(

            CompressedImage, topic_name, qos_profile)
 
        # Initialize the CvBridge

        self.bridge = CvBridge()
 
        # Load the image

        self.image = cv2.imread(image_path)

        if self.image is None:

            self.get_logger().error(f"Failed to load image: {image_path}")

            return
 
        # Initialize the message counter for sequential frame_id

        self.message_count = 1
 
        # Publish rate setup

        self.publish_rate = publish_rate

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_image)
 
    def publish_image(self):

        try:

            # Encode the image as JPEG

            success, encoded_image = cv2.imencode('.jpg', self.image)

            if not success:

                raise RuntimeError("Image encoding failed")
 
            # Create CompressedImage message

            ros_image = CompressedImage()

            ros_image.header.stamp = self.get_clock().now().to_msg()

            ros_image.header.frame_id = str(self.message_count)

            ros_image.format = "jpeg"

            ros_image.data = encoded_image.tobytes()
 
            # Publish the message

            self.publisher_.publish(ros_image)

            self.get_logger().info(f"Published compressed image with frame_id {self.message_count} at {self.publish_rate} Hz")
 
            self.message_count += 1
 
        except Exception as e:

            self.get_logger().error(f"Error publishing compressed image: {e}")
 
def main(args=None):

    rclpy.init(args=args)

    node = ImagePublisherNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()

 
