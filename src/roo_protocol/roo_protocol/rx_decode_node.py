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
        self.pub_v = self.create_publisher(Float32, '/roo/telemetry/voltage_v', 20)
        self.pub_i = self.create_publisher(Float32, '/roo/telemetry/current_a', 20)
        self.pub_enc_f = self.create_publisher(Float32, '/roo/telemetry/encoder_front_deg', 20)
        self.pub_enc_b = self.create_publisher(Float32, '/roo/telemetry/encoder_back_deg', 20)
        self.pub_pitch = self.create_publisher(Float32, '/roo/telemetry/pitch_deg', 20)
        self.pub_roll = self.create_publisher(Float32, '/roo/telemetry/roll_deg', 20)

        # Subscription
        self.sub = self.create_subscription(String, rx_topic, self.on_frame, 50)

        self.get_logger().info(f"RxDecodeNode listening on {rx_topic}")

    def on_frame(self, msg: String):
        s = msg.data.strip()
        m = FRAME_RE.match(s)
        if not m:
            # Ignore non-framed noise
            return

        dev_id = int(m.group(1))
        payload = m.group(2)

        # ID 10: INA226, payload "V12.34-I0.56"
        if dev_id == 10:
            # Very tolerant parse
            # Accept "V..-I.." or "V.. I.." or "V..,I.."
            v = None
            i = None
            # Look for V<number>
            mv = re.search(r'V\s*([-+]?\d+(\.\d+)?)', payload)
            mi = re.search(r'I\s*([-+]?\d+(\.\d+)?)', payload)
            if mv:
                v = safe_float(mv.group(1))
            if mi:
                i = safe_float(mi.group(1))

            if v is not None:
                out = Float32(); out.data = float(v)
                self.pub_v.publish(out)
            if i is not None:
                out = Float32(); out.data = float(i)
                self.pub_i.publish(out)
            return

        # ID 11: Encoders, payload "F180B92"
        if dev_id == 11:
            mf = re.search(r'F\s*([-+]?\d+(\.\d+)?)', payload)
            mb = re.search(r'B\s*([-+]?\d+(\.\d+)?)', payload)
            if mf:
                val = safe_float(mf.group(1))
                if val is not None:
                    out = Float32(); out.data = float(val)
                    self.pub_enc_f.publish(out)
            if mb:
                val = safe_float(mb.group(1))
                if val is not None:
                    out = Float32(); out.data = float(val)
                    self.pub_enc_b.publish(out)
            return

        # ID 12: Tilt, payload "P12R-3"
        if dev_id == 12:
            mp = re.search(r'P\s*([-+]?\d+(\.\d+)?)', payload)
            mr = re.search(r'R\s*([-+]?\d+(\.\d+)?)', payload)
            if mp:
                val = safe_float(mp.group(1))
                if val is not None:
                    out = Float32(); out.data = float(val)
                    self.pub_pitch.publish(out)
            if mr:
                val = safe_float(mr.group(1))
                if val is not None:
                    out = Float32(); out.data = float(val)
                    self.pub_roll.publish(out)
            return


def main():
    rclpy.init()
    node = RxDecodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
