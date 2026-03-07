#!/usr/bin/env python3
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

FRAME_RE = re.compile(r'^\$(\d+)~(.*)`$')

def safe_float(s: str):
    try:
        return float(s)
    except Exception:
        return None

class RxDecodeNode(Node):
    def __init__(self):
        super().__init__('rx_decode_node')

        # Parameters
        self.declare_parameter('rx_buffer_topic', '/roo/rx_buffer')
        rx_topic = self.get_parameter('rx_buffer_topic').get_parameter_value().string_value

        # Publishers (telemetry)
        self.pub_pitch = self.create_publisher(Float32, '/roo/telemetry/pitch_deg', 20)
        self.pub_roll = self.create_publisher(Float32, '/roo/telemetry/roll_deg', 20)
        
        # Power Publishers (ID 14)
        self.pub_volt = self.create_publisher(Float32, '/roo/battery/voltage', 10)
        self.pub_curr = self.create_publisher(Float32, '/roo/battery/current', 10)
        self.pub_energy = self.create_publisher(Float32, '/roo/battery/energy', 10)

        # Encoder Publishers (ID 15)
        self.pub_enc1 = self.create_publisher(Float32, '/roo/gimbal/enc1_angle', 10)
        self.pub_enc2 = self.create_publisher(Float32, '/roo/gimbal/enc2_angle', 10)

        # Subscription
        self.sub = self.create_subscription(String, rx_topic, self.on_frame, 50)
        self.get_logger().info(f"RxDecodeNode listening on {rx_topic}")

    def on_frame(self, msg: String):
        s = msg.data.strip()
        m = FRAME_RE.match(s)
        if not m:
            return

        dev_id = int(m.group(1))
        payload = m.group(2)

        # ID 12: Tilt (Legacy GY-85 format)
        if dev_id == 12:
            mp = re.search(r'P\s*([-+]?\d+(\.\d+)?)', payload)
            mr = re.search(r'R\s*([-+]?\d+(\.\d+)?)', payload)
            if mp:
                val = safe_float(mp.group(1))
                if val is not None: self.pub_pitch.publish(Float32(data=float(val)))
            if mr:
                val = safe_float(mr.group(1))
                if val is not None: self.pub_roll.publish(Float32(data=float(val)))

        # ID 14: Power Telemetry (V, I, E) - CSV format
        elif dev_id == 14:
            parts = payload.split(',')
            if len(parts) >= 3:
                v = safe_float(parts[0])
                i = safe_float(parts[1])
                e = safe_float(parts[2])
                if v is not None: self.pub_volt.publish(Float32(data=v))
                if i is not None: self.pub_curr.publish(Float32(data=i))
                if e is not None: self.pub_energy.publish(Float32(data=e))

        # ID 15: Encoder Feedback (Enc1, Enc2) - CSV format
        elif dev_id == 15:
            parts = payload.split(',')
            if len(parts) >= 2:
                e1 = safe_float(parts[0])
                e2 = safe_float(parts[1])
                if e1 is not None: self.pub_enc1.publish(Float32(data=e1))
                if e2 is not None: self.pub_enc2.publish(Float32(data=e2))

def main():
    rclpy.init()
    node = RxDecodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
