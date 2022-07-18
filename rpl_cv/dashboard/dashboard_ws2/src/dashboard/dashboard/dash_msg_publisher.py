import socket
import struct

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from std_msgs.msg import String


# Client
class DashMessagePublisher(Node):
    """Establishes connection with Flask Dashboard App with a socket connection.
    It receives messages from the socket connection and it then publishes these
    messages to its respective ros_topic (the data sent over the socket server says
    what ros topic it wants the message to be sent on)

    Args:
        Node (rply.node.Node): ROS Node parent class
    """

    def __init__(self, topic, ip_addr: str = "127.0.0.1", port=9080):
        super().__init__("dash_msg_publisher")

        self.publisher = self.create_publisher(String, topic, 10)
        self.topic = topic

        self.ip = ip_addr
        self.port = port
        self.addr = (ip_addr, port)

        self.sock = socket.create_connection((ip_addr, port))
        self.sock_listen()

    def read_bytes(self, size):
        buf_list = []
        while sum(len(b) for b in buf_list) < size:
            packet = self.sock.recv(4096)  # receive data in chunks of 4096 bytes
            if not packet:
                raise ConnectionError
            buf_list.append(packet)

        return b"".join(buf_list)

    def sock_listen(self):
        header_size = struct.calcsize("Q")
        buffer = b""

        try:
            while True:
                buffer += self.read_bytes(header_size - len(buffer))
                header, buffer = buffer[:header_size], buffer[header_size:]
                data_size = struct.unpack("Q", header)[0]

                # read the data
                buffer += self.read_bytes(data_size - len(buffer))

                # split the messsage from the front of the buffer, preserve the rest of the buffer
                msg, buffer = buffer[:data_size], buffer[data_size:]

                print(str(msg))  # just checking if it works

                # TODO: send msg to ros topic

        except ConnectionError:
            print("Lost connection from:", self.addr)
        except KeyboardInterrupt:
            print("Got KeyboardInterrupt. Closing socket.")
        finally:
            self.sock.close()


def main(args=None):
    rclpy.init(args=args)

    msg_publisher = DashMessagePublisher(topic="dash_msgs")
    rclpy.spin(msg_publisher)

    msg_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
