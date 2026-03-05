#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialRxNode(Node):
    def __init__(self):
        super().__init__('serial_rx_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/roo/rx_buffer')
        self.declare_parameter('frame_end', '`')   # end-of-frame char
        self.declare_parameter('read_chunk', 64)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_end = self.get_parameter('frame_end').get_parameter_value().string_value
        self.read_chunk = self.get_parameter('read_chunk').get_parameter_value().integer_value

        self.pub = self.create_publisher(String, self.topic, 50)

        self.ser = None
        self.buf = ""

        self.timer = self.create_timer(0.01, self.loop)  # 100 Hz poll

        self.get_logger().info(f"SerialRxNode starting: port={self.port} baud={self.baud} topic={self.topic}")

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info("Serial connected")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Serial connect failed: {e}")

    def loop(self):
        if self.ser is None or not self.ser.is_open:
            self.connect()
            return

        try:
            data = self.ser.read(self.read_chunk)
            if not data:
                return

            text = data.decode('utf-8', errors='ignore')
            self.buf += text

            # Extract complete frames ending with frame_end
            end = self.frame_end
            while end in self.buf:
                frame, self.buf = self.buf.split(end, 1)
                if frame == "":
                    continue
                msg = String()
                msg.data = frame + end   # publish INCLUDING the end char
                self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            time.sleep(0.2)

def main():
    rclpy.init()
    node = SerialRxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
