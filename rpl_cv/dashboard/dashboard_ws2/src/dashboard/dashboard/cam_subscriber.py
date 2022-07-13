import socket
import struct

import cv2  # OpenCV library
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String


class CameraSubscriber(Node):
    """
    CameraSubscriber is a node that subscribes to a topic, and publishes the frames
    that it receives from the topic to a socket connection.
    """

    def __init__(self, topic: str, ip_addr: str = "127.0.0.1", port: int = 9999):
        """Class Constructor to setup socket connection and to subscribe to a ros topic

        Args:
            topic (str):                The ros topic that this node should subscribe to
            ip_conn (str, optional):    IP Address that you want to connect to send the frames to. Defaults to "127.0.0.1".
            port (int, optional):       Port for the IP address you wish to connect to. Defaults to 9999.
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("camera_subscriber")

        self.subscription = self.create_subscription(
            Image, topic, self.listener_callback, 10
        )
        self.topic = topic
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.i = 0  # used to count the frames
        self.ip = ip_addr
        self.port = port
        self.sock = socket.create_connection((ip_addr, port))

    def listener_callback(self, data: Image):
        """Callback function that's called anytime new data is received on the ros topic

        Args:
            data (Image): The ROS image that is received on the ros topic
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Display image ---> not displaying the images here since we are sending over socket connection
        # cv2.imshow("camera", current_frame)
        # cv2.imwrite(f"Frame_{self.i}.jpg", current_frame)
        # cv2.waitKey(1)

        # Send image over socket connection
        self._send_frame(current_frame)
        self.get_logger().info(f"Sent video frame {self.i} to {self.ip}:{self.port}")

        self.i += 1

    def _send_frame(self, img, quality=95):
        """Helper function to send frame over socket connection

        Args:
            img (OpenCV Image):         OpenCV Image (i.e., np array)
            quality (int, optional):    Amount of quality to retain when compressing image using JPEG compression.
                                        Defaults to 95.
        """
        _, img = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        data = img.tobytes()
        header = struct.pack("Q", len(data))
        self.sock.sendall(header + data)

    def close_socket(self):
        self.sock.close()


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = CameraSubscriber(topic="video_frames")

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.close_socket()
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
