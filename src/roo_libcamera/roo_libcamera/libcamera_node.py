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

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 480)
        self.declare_parameter('height', 360)
        self.declare_parameter('framerate', 10)
        self.declare_parameter('image_topic', '/roo/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publish compressed JPEG to topic/compressed (avoids decode, ~20x smaller)
        compressed_topic = image_topic.rstrip('/') + '/compressed'
        self.pub = self.create_publisher(CompressedImage, compressed_topic, 1)
        self._proc = None
        self._running = False
        self._read_thread = None
        self._stderr_thread = None

        self.get_logger().info(
            f'libcamera_node (cam {self.camera_id}): {self.width}x{self.height} @ {self.framerate}fps -> {compressed_topic} (compressed)'
        )

    def start_capture(self):
        """Capture from the specified Pi CSI camera using rpicam-vid."""
        cmd = [
            'rpicam-vid',
            '--camera', str(self.camera_id),
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.framerate),
            '--codec', 'mjpeg',
            '-t', '0',  # Run forever
            '--nopreview',
            '-o', '-',  # Output to stdout
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, # Capture stderr to see errors from rpicam-vid
                bufsize=0, # Unbuffered pipe for minimum lag
            )
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            # Also start a thread to monitor stderr for debugging
            self._stderr_thread = threading.Thread(target=self._stderr_loop, daemon=True)
            self._stderr_thread.start()
            self.get_logger().info(f"'{cmd[0]}' process started for camera {self.camera_id}.")
        except FileNotFoundError:
            self.get_logger().error(f"Failed to start capture: '{cmd[0]}' command not found. Is libcamera-apps installed? (e.g. 'sudo apt install libcamera-apps')")
        except Exception as e:
            self.get_logger().error(f"Failed to start '{cmd[0]}': {e}")

    def _stderr_loop(self):
        """Read from the subprocess stderr and log it for debugging."""
        if not self._proc or not self._proc.stderr:
            return
        try:
            for line in iter(self._proc.stderr.readline, b''):
                if not self._running:
                    break
                self.get_logger().warn(f"[rpicam-vid stderr] {line.decode('utf-8').strip()}")
        except Exception as e:
            if self._running:
                self.get_logger().error(f"Error in stderr loop: {e}")

    def _read_loop(self):
        """High-speed stream parser."""
        SOI = b'\xff\xd8'
        EOI = b'\xff\xd9'
        buf = b''

        while self._running and self._proc and self._proc.stdout:
            # Reading in chunks is more efficient than one byte at a time
            chunk = self._proc.stdout.read(65536)
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
        if self._read_thread:
            self._read_thread.join(timeout=1)
            self._read_thread = None
        if self._stderr_thread:
            self._stderr_thread.join(timeout=1)
            self._stderr_thread = None

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
