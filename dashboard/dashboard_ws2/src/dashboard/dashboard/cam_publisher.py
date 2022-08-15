import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type


class CameraPublisher(Node):
    """
    Create an CameraPublisher class, which is a subclass of the Node class.
    Responsible for publishing camera frames to ro
    """

    def __init__(self, topic):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("camera_publisher")

        # Create the publisher. This publisher will publish an Image
        # to the topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.topic = topic
        timer_period = 0.1  # seconds - publish every 0.1 seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.i = 0

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info("Publishing video frame %d" % self.i)
        self.i += 1


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    cam_publisher = CameraPublisher(topic="video_frames")

    # Spin the node so the callback function is called.
    rclpy.spin(cam_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
