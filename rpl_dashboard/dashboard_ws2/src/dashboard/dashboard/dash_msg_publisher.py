import select
import socket
import struct
from typing import Tuple

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from std_msgs.msg import String


# SERVER
class DashMessagePublisher(Node):
    """Establishes connection with Flask Dashboard App with a socket connection.
    It receives messages from the socket connection and it then publishes these
    messages to its respective ros_topic (the data sent over the socket server says
    what ros topic it wants the message to be sent on)

    Args:
        Node (rply.node.Node): ROS Node parent class
    """

    def __init__(self, topic, sock_client):
        super().__init__("dash_msg_publisher")

        self.publisher = self.create_publisher(String, topic, 10)
        self.topic = topic

        self.sock_client = sock_client
        self.timeout = 0.1  # timeout in seconds

    def read_bytes(self, size):
        buf_list = []
        while sum(len(b) for b in buf_list) < size:
            packet = self.sock_client.recv(4096)  # receive data in chunks of 4096 bytes
            if not packet:
                raise ConnectionError
            buf_list.append(packet)

        return b"".join(buf_list)

    def sock_listen(self) -> bool:
        header_size = struct.calcsize("Q")
        buffer = b""

        try:
            ready = select.select([self.sock_client], [], [], self.timeout)
            if not ready[0]:
                return True  # nothing on the socket to be grabbed

            buffer += self.read_bytes(header_size - len(buffer))
            header, buffer = buffer[:header_size], buffer[header_size:]
            data_size = struct.unpack("Q", header)[0]

            # read the data
            buffer += self.read_bytes(data_size - len(buffer))

            # split the messsage from the front of the buffer, preserve the rest of the buffer
            socket_message, buffer = buffer[:data_size], buffer[data_size:]
            message = String()
            message.data = str(socket_message)

            self.publisher.publish(message)
            print(
                "Published Message: %s" % str(socket_message)
            )  # just checking if it works

        except ConnectionError:
            print("Lost connection from:", self.addr)
            self.sock_client.close()
            return False
        except KeyboardInterrupt:
            print("Got KeyboardInterrupt. Closing socket.")
            self.sock_client.close()
            return False

        return True


def create_socket_connection(ip: str, port: int) -> Tuple[object, object, str]:
    """Establishes a socket connection w/ (ip, port).

    Args:
        ip (str): IP Address
        port (int): Port

    Returns:
        Tuple: returns the sock, sock_client, and addr
    """
    # SOCK_STREAM = TCP Connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.listen()
    print(f"Listening at ip:{ip} port:{port}")

    sock_client, addr = sock.accept()
    if not sock_client:
        return
    print(f"Got connection from: {addr}")

    return sock, sock_client, addr


def main(args=None):
    sock, sock_client, addr = create_socket_connection(ip="127.0.0.1", port=9080)
    ros_topic = "dash_msgs"
    rclpy.init(args=args)

    msg_publisher = DashMessagePublisher(ros_topic, sock_client)

    while True:
        if not msg_publisher.sock_listen():
            print("Failed to listen for messages. Exiting.")
            break
        rclpy.spin_once(msg_publisher, timeout_sec=0.1)

    msg_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
