import rclpy
from rclpy.node import Node

from std_msgs.msg import Byte

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscription = self.create_subscription(
            Byte,
            'camera_topic',
            self.listener_callback,
            45
        )
        self.subscription   # prevent unused variable warning
        self.i = 0;

    def listener_callback(self, msg):
        self.get_logger().info("Received Frame %d" % self.i)
        self.i += 1

        # frame = msg.data
        # TODO: publish data to dashboard or send to dashboard.py


def main(args=None):
    rclpy.init(args=args)

    cam_sub = CameraSubscriber()

    rclpy.spin(cam_sub)

    # Destroy node explicitly
    # (optimal - otherwise it's done by garbage collector)
    cam_sub.destory_node()
    rclpy.shutdown()