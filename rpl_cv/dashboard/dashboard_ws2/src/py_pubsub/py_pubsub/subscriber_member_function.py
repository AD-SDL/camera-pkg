# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

import socket
import struct
 
class CameraSubscriber(Node):
    """
    Create an CameraSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self, ip_conn='127.0.0.1', port=8080):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
            
        self.subscription = self.create_subscription(
            Image, 
            'video_frames', 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.i = 0

        self.sock = socket.create_connection((ip, port))

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        self.get_logger().info('Received video frame %d' % self.i)
        # Display image
        # cv2.imshow("camera", current_frame)
        # cv2.imwrite(f"Frame_{self.i}.jpg", current_frame)
        # cv2.waitKey(1)

        # Send image over socket connection
        self.send_frame(current_frame)

        self.i += 1

    def send_frame(self, img, quality=95):
        _, img = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        data = img.tobytes()
        header = struct.pack('Q', len(data))
        self.sock.sendall(header + data)

    def close_socket(self):
        self.sock.close()

  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = CameraSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
