import socket

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from std_msgs.msg import String


class DashMessagePublisher(Node):
    """Establishes connection with Flask Dashboard App with a socket connection.
    It receives messages from the socket connection and it then publishes these
    messages to its respective ros_topic (the data sent over the socket server says
    what ros topic it wants the message to be sent on)

    Args:
        Node (rply.node.Node): ROS Node parent class
    """

    # TODO: is it okay to have same port as the CameraSubscriber stuff
    def __init__(self, topic, ip_addr: str = "127.0.0.1", port=9999):
        super().__init__("dash_msg_publisher")

        self.publisher = self.create_publisher(String, topic, 10)
        self.topic = topic

        self.ip = ip_addr
        self.port = port
        self.sock = socket.create_connection((ip_addr, port))

    def sock_listen():
        pass
