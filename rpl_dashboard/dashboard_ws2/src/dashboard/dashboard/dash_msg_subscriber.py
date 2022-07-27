import cv2  # OpenCV library
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String


class CameraSubscriber(Node):
    def __init__(self, topic: str):

        super().__init__("dash_msg_subscriber")
        self.subscription = self.create_subscription(
            String, topic, self.listener_callback, 10
        )
        self.topic = topic
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: String):
        self.get_logger().info("Received message: %s" % str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber(topic="dash_msgs")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
