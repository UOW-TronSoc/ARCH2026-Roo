#!/usr/bin/env python3
"""
Libcamera-native ROS2 camera node.
Uses rpicam-vid with MJPEG to capture directly from PiSP libcamera,
bypassing the V4L2 compat layer that causes green banding/corruption.
Publishes CompressedImage (JPEG) to reduce bandwidth; use compressed_republisher
for raw Image (e.g. web_video_server).
"""
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class LibcameraNode(Node):
    def __init__(self):
        super().__init__('libcamera_node')

        self.declare_parameter('width', 480)
        self.declare_parameter('height', 360)
        self.declare_parameter('framerate', 10)
        self.declare_parameter('image_topic', '/roo/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('camera_id', 0)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value

        # Publish compressed JPEG to topic/compressed (avoids decode, ~20x smaller)
        compressed_topic = image_topic.rstrip('/') + '/compressed'
        self.pub = self.create_publisher(CompressedImage, compressed_topic, 1)
        self._proc = None
        self._running = False
        self._thread = None

        self.get_logger().info(
            f'libcamera_node: {self.width}x{self.height} @ {self.framerate}fps -> {compressed_topic} (compressed)'
        )

    def start_capture(self):
        """Stable capture without risky priority flags."""
        cmd = [
            'rpicam-vid',
            '--camera', str(self.camera_id),
            '-o', '-',
            '-t', '0',
            '-n',
            '--codec', 'mjpeg',
            '--width', '480',
            '--height', '360',
            '--framerate', '15',
            '--quality', '40',
            '--shutter', '10000',
            '--flush', '1' # Forces the OS to send data immediately
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0, # Unbuffered pipe for minimum lag
            )
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.get_logger().info(f'Camera {self.camera_id} online.')
        except Exception as e:
            self.get_logger().error(f'Failed: {e}')

    def _read_loop(self):
        """High-speed stream parser."""
        SOI = b'\xff\xd8'
        EOI = b'\xff\xd9'
        buf = b''

        while self._running and self._proc and self._proc.stdout:
            # Small reads prevent the 'bursting' issue we saw earlier
            chunk = self._proc.stdout.read(4096)
            if not chunk:
                break
            buf += chunk

            # Find the MOST RECENT frame in the current buffer
            # This is the secret to zero latency.
            while True:
                start = buf.find(SOI)
                if start < 0:
                    buf = buf[-1:] if buf else b''
                    break
                
                end = buf.find(EOI, start)
                if end < 0:
                    buf = buf[start:]
                    break

                # We found a full frame!
                end += 2
                jpeg_data = buf[start:end]
                buf = buf[end:]

                # If there's ANOTHER frame already starting in the buffer, 
                # skip this one to catch up to real-time.
                if buf.find(SOI) >= 0:
                    continue 

                msg = CompressedImage()
                msg.format = 'jpeg'
                msg.data = bytes(jpeg_data)
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.pub.publish(msg)
    def stop_capture(self):
        self._running = False
        if self._proc:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=2)
            except Exception:
                self._proc.kill()
            self._proc = None
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def destroy_node(self):
        self.stop_capture()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LibcameraNode()
    node.start_capture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_capture()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
