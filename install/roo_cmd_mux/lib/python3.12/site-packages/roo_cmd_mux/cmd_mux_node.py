#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from dataclasses import dataclass
from typing import Dict, Tuple
import time

@dataclass
class CmdState:
    motor_left: float = 0.0
    motor_right: float = 0.0
    gimbal2: float = 90.0
    gimbal3: float = 90.0
    susp_front: float = 0.0
    susp_back: float = 0.0
    last_update: float = 0.0

class CmdMuxNode(Node):
    """
    Mux between joystick and website command topics.

    Inputs:
      /roo/joy_cmd/*   (Float32)
      /roo/web_cmd/*   (Float32)
      /roo/control_source (String): "joy" or "web"

    Output (canonical):
      /roo/cmd/* (Float32)

    Safety:
      If selected source is stale (>timeout_s), motors+susp go to 0.
    """

    def __init__(self):
        super().__init__('cmd_mux_node')

        # Parameters
        self.declare_parameter('source_topic', '/roo/control_source')
        self.declare_parameter('default_source', 'joy')  # "joy" or "web"
        self.declare_parameter('timeout_s', 0.35)        # stale timeout
        self.declare_parameter('publish_hz', 50.0)       # output rate

        self.source_topic = self.get_parameter('source_topic').value
        self.selected = self.get_parameter('default_source').value
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        # State for each source
        self.state: Dict[str, CmdState] = {
            'joy': CmdState(last_update=0.0),
            'web': CmdState(last_update=0.0),
        }

        # Publishers (canonical outputs)
        self.pub_ml = self.create_publisher(Float32, '/roo/cmd/motor_left', 20)
        self.pub_mr = self.create_publisher(Float32, '/roo/cmd/motor_right', 20)
        self.pub_g2 = self.create_publisher(Float32, '/roo/cmd/gimbal_servo_2', 20)
        self.pub_g3 = self.create_publisher(Float32, '/roo/cmd/gimbal_servo_3', 20)
        self.pub_sf = self.create_publisher(Float32, '/roo/cmd/suspension_front', 20)
        self.pub_sb = self.create_publisher(Float32, '/roo/cmd/suspension_back', 20)

        # Source selection subscriber
        self.create_subscription(String, self.source_topic, self.on_source, 10)

        # Joy inputs
        self.create_subscription(Float32, '/roo/joy_cmd/motor_left', self._mk_cb('joy', 'motor_left'), 20)
        self.create_subscription(Float32, '/roo/joy_cmd/motor_right', self._mk_cb('joy', 'motor_right'), 20)
        self.create_subscription(Float32, '/roo/joy_cmd/gimbal_servo_2', self._mk_cb('joy', 'gimbal2'), 20)
        self.create_subscription(Float32, '/roo/joy_cmd/gimbal_servo_3', self._mk_cb('joy', 'gimbal3'), 20)
        self.create_subscription(Float32, '/roo/joy_cmd/suspension_front', self._mk_cb('joy', 'susp_front'), 20)
        self.create_subscription(Float32, '/roo/joy_cmd/suspension_back', self._mk_cb('joy', 'susp_back'), 20)

        # Web inputs (for later)
        self.create_subscription(Float32, '/roo/web_cmd/motor_left', self._mk_cb('web', 'motor_left'), 20)
        self.create_subscription(Float32, '/roo/web_cmd/motor_right', self._mk_cb('web', 'motor_right'), 20)
        self.create_subscription(Float32, '/roo/web_cmd/gimbal_servo_2', self._mk_cb('web', 'gimbal2'), 20)
        self.create_subscription(Float32, '/roo/web_cmd/gimbal_servo_3', self._mk_cb('web', 'gimbal3'), 20)
        self.create_subscription(Float32, '/roo/web_cmd/suspension_front', self._mk_cb('web', 'susp_front'), 20)
        self.create_subscription(Float32, '/roo/web_cmd/suspension_back', self._mk_cb('web', 'susp_back'), 20)

        # Publish timer
        period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_loop)

        self.get_logger().info(
            f"CmdMux started. default_source='{self.selected}', source_topic='{self.source_topic}', "
            f"timeout_s={self.timeout_s}, publish_hz={self.publish_hz}"
        )

    def on_source(self, msg: String):
        s = msg.data.strip().lower()
        if s not in ('joy', 'web'):
            self.get_logger().warn(f"Ignoring unknown source '{msg.data}'. Use 'joy' or 'web'.")
            return
        if s != self.selected:
            self.selected = s
            self.get_logger().info(f"Control source switched to: {self.selected}")

    def _mk_cb(self, source: str, field: str):
        def cb(msg: Float32):
            st = self.state[source]
            setattr(st, field, float(msg.data))
            st.last_update = time.time()
        return cb

    def publish_loop(self):
        now = time.time()
        st = self.state[self.selected]
        stale = (now - st.last_update) > self.timeout_s if st.last_update > 0.0 else True

        # Safety behaviour:
        # - If stale: motors+susp = 0, keep last gimbal angles (so camera doesn't jump)
        if stale:
            motor_left = 0.0
            motor_right = 0.0
            susp_front = 0.0
            susp_back = 0.0
            g2 = st.gimbal2
            g3 = st.gimbal3
        else:
            motor_left = st.motor_left
            motor_right = st.motor_right
            susp_front = st.susp_front
            susp_back = st.susp_back
            g2 = st.gimbal2
            g3 = st.gimbal3

        self._pub(self.pub_ml, motor_left)
        self._pub(self.pub_mr, motor_right)
        self._pub(self.pub_sf, susp_front)
        self._pub(self.pub_sb, susp_back)
        self._pub(self.pub_g2, g2)
        self._pub(self.pub_g3, g3)

    def _pub(self, pub, value: float):
        m = Float32()
        m.data = float(value)
        pub.publish(m)

def main():
    rclpy.init()
    node = CmdMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
