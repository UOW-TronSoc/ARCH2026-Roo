#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import serial
import struct
import time
import cv2
import numpy as np
from cv_bridge import CvBridge

MAGIC = b'ROO1'

class UartCamNode(Node):
    def __init__(self):
        super().__init__('uart_cam_node')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 1500000)
        self.declare_parameter('topic', '/cam0/image_raw')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        self.get_logger().info(f"Opening UART camera port={port} baud={baud}")
        self.ser = serial.Serial(port, baud, timeout=0.2)

        self.buf = bytearray()
        self.last_pub = time.time()

        self.timer = self.create_timer(0.001, self.spin_once)

    def spin_once(self):
        try:
            data = self.ser.read(4096)
        except Exception as e:
            self.get_logger().warn(f"serial read error: {e}")
            return
        if data:
            self.buf.extend(data)

        # Find MAGIC
        while True:
            idx = self.buf.find(MAGIC)
            if idx < 0:
                # keep buffer bounded
                if len(self.buf) > 1_000_000:
                    self.buf = self.buf[-10000:]
                return
            if idx > 0:
                del self.buf[:idx]
            # need header: MAGIC(4) + len(4)
            if len(self.buf) < 8:
                return
            length = struct.unpack('<I', self.buf[4:8])[0]
            if length == 0 or length > 2_000_000:
                # bad length, drop magic
                del self.buf[:4]
                continue
            if len(self.buf) < 8 + length:
                return

            jpeg = bytes(self.buf[8:8+length])
            del self.buf[:8+length]

            # decode JPEG -> bgr
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue

            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'cam0'
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = UartCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.ser.close()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
