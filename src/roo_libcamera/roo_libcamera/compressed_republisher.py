#!/usr/bin/env python3
"""
Subscribes to CompressedImage, decompresses to Image, republishes for consumers
that need raw (e.g. web_video_server).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class CompressedRepublisher(Node):
    def __init__(self):
        super().__init__('compressed_republisher')

        self.declare_parameter('compressed_topic', '/roo/image_raw/compressed')
        self.declare_parameter('image_topic', '/roo/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        compressed_topic = self.get_parameter('compressed_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.sub = self.create_subscription(
            CompressedImage,
            compressed_topic,
            self._callback,
            10,
        )
        self.pub = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()

        self.get_logger().info(
            f'compressed_republisher: {compressed_topic} -> {image_topic}'
        )

    def _callback(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                out.header = msg.header
                if not out.header.frame_id:
                    out.header.frame_id = self.frame_id
                self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f'Decompress error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CompressedRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
