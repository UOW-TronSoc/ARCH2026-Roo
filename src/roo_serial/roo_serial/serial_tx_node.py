#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialTxNode(Node):
    def __init__(self):
        super().__init__('serial_tx_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/roo/tx_buffer')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.sub = self.create_subscription(String, self.topic, self.on_msg, 50)

        self.ser = None
        self.get_logger().info(f"SerialTxNode starting: port={self.port} baud={self.baud} topic={self.topic}")

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info("Serial connected")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Serial connect failed: {e}")

    def on_msg(self, msg: String):
        if self.ser is None or not self.ser.is_open:
            self.connect()
            if self.ser is None:
                return

        try:
            out = msg.data.encode('utf-8')
            self.ser.write(out)
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            time.sleep(0.2)

def main():
    rclpy.init()
    node = SerialTxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
