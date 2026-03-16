#!/usr/bin/env python3
"""
Subscribes to raw Image, compresses to JPEG, and republishes as CompressedImage.
This is useful for camera drivers that only output raw images, to prepare
the stream for consumers that prefer compressed topics (e.g. web_video_server).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')

        self.declare_parameter('image_topic', '/image_raw')
        # The output topic is derived from the input topic
        self.declare_parameter('jpeg_quality', 90)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        compressed_topic = image_topic.rstrip('/') + '/compressed'

        self.bridge = CvBridge()
        self.pub = self.create_publisher(CompressedImage, compressed_topic, 10)
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self._callback,
            10,
        )

        self.get_logger().info(
            f'Image compressor: {image_topic} -> {compressed_topic} (jpeg quality: {self.jpeg_quality})'
        )

    def _callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Prepare JPEG compression parameters
            params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

            # Compress to JPEG
            result, compressed_data = cv2.imencode('.jpg', cv_image, params)
            
            if not result:
                self.get_logger().warn('cv2.imencode failed')
                return

            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = compressed_data.tobytes()
            self.pub.publish(out)
        except CvBridgeError as e:
            self.get_logger().warn(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().warn(f'Compression error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()