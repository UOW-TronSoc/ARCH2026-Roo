#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class TeleopPS5Node(Node):
    """
    PS5 DualSense -> Roo command topics (per your mapping)

    Deadman: X button must be held for motors + suspension + gimbal motion updates.

    Driving:
      left stick: throttle axis, turn axis -> motor left/right

    Gimbal (2DOF):
      right stick: integrates into servo angles
        gimbal_2 (about Y) -> /roo/cmd/gimbal_servo_2
        gimbal_3 (about X) -> /roo/cmd/gimbal_servo_3

    Suspension:
      front: L1 up, L2 down
      back:  R1 up, R2 down
      publishes speed commands in range [-max_cmd, +max_cmd]
    """

    def __init__(self):
        super().__init__('teleop_ps5_node')

        # ----- Topics -----
        self.declare_parameter('joy_topic', '/joy')

        # Publish *joy-specific* command topics (we'll mux later)
        self.declare_parameter('motor_left_topic',  '/roo/joy_cmd/motor_left')
        self.declare_parameter('motor_right_topic', '/roo/joy_cmd/motor_right')

        self.declare_parameter('gimbal2_topic', '/roo/joy_cmd/gimbal_servo_2')
        self.declare_parameter('gimbal3_topic', '/roo/joy_cmd/gimbal_servo_3')

        self.declare_parameter('susp_front_topic', '/roo/joy_cmd/suspension_front')
        self.declare_parameter('susp_back_topic',  '/roo/joy_cmd/suspension_back')

        # ----- Ranges / Gains -----
        self.declare_parameter('max_cmd', 10.0)  # motors/susp speed range -10..+10
        self.declare_parameter('throttle_gain', 1.0)
        self.declare_parameter('turn_gain', 1.0)

        # Gimbal behaviour
        self.declare_parameter('gimbal_rate_deg_s', 90.0)  # stick=1.0 -> 90 deg/s
        self.declare_parameter('gimbal_min_deg', 0.0)
        self.declare_parameter('gimbal_max_deg', 180.0)
        self.declare_parameter('gimbal2_start_deg', 90.0)
        self.declare_parameter('gimbal3_start_deg', 90.0)

        # Suspension speed when button held
        self.declare_parameter('susp_speed', 6.0)  # magnitude in -10..+10

        # ----- Axis indices -----
        # left stick
        self.declare_parameter('throttle_axis', 1)  # left stick Y
        self.declare_parameter('turn_axis', 0)      # left stick X

        # right stick
        self.declare_parameter('gimbal2_axis', 3)   # right stick X/Y can vary; you can swap
        self.declare_parameter('gimbal3_axis', 4)

        # trigger axes (if your L2/R2 are axes with 1.0 unpressed, -1.0 pressed)
        self.declare_parameter('l2_axis', 2)
        self.declare_parameter('r2_axis', 5)

        # ----- Button indices -----
        # Deadman (X / Cross). You MUST set this correctly if your mapping differs.
        self.declare_parameter('deadman_button', 1)  # common DualSense mapping, may differ
        self.declare_parameter('l1_button', 4)       # common DualSense mapping, may differ
        self.declare_parameter('r1_button', 5)       # common DualSense mapping, may differ

        # Whether L2/R2 are read as axes (preferred) or buttons
        self.declare_parameter('l2_is_axis', True)
        self.declare_parameter('r2_is_axis', True)
        self.declare_parameter('l2_button', 6)  # fallback if trigger is button
        self.declare_parameter('r2_button', 7)

        joy_topic = self.get_parameter('joy_topic').value

        self.max_cmd = float(self.get_parameter('max_cmd').value)
        self.throttle_gain = float(self.get_parameter('throttle_gain').value)
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.susp_speed = float(self.get_parameter('susp_speed').value)

        self.throttle_axis = int(self.get_parameter('throttle_axis').value)
        self.turn_axis = int(self.get_parameter('turn_axis').value)
        self.gimbal2_axis = int(self.get_parameter('gimbal2_axis').value)
        self.gimbal3_axis = int(self.get_parameter('gimbal3_axis').value)
        self.l2_axis = int(self.get_parameter('l2_axis').value)
        self.r2_axis = int(self.get_parameter('r2_axis').value)

        self.deadman_button = int(self.get_parameter('deadman_button').value)
        self.l1_button = int(self.get_parameter('l1_button').value)
        self.r1_button = int(self.get_parameter('r1_button').value)

        self.l2_is_axis = bool(self.get_parameter('l2_is_axis').value)
        self.r2_is_axis = bool(self.get_parameter('r2_is_axis').value)
        self.l2_button = int(self.get_parameter('l2_button').value)
        self.r2_button = int(self.get_parameter('r2_button').value)

        # Gimbal state
        self.gimbal_rate = float(self.get_parameter('gimbal_rate_deg_s').value)
        self.gimbal_min = float(self.get_parameter('gimbal_min_deg').value)
        self.gimbal_max = float(self.get_parameter('gimbal_max_deg').value)
        self.g2 = float(self.get_parameter('gimbal2_start_deg').value)
        self.g3 = float(self.get_parameter('gimbal3_start_deg').value)

        # Publishers
        self.pub_ml = self.create_publisher(Float32, self.get_parameter('motor_left_topic').value, 20)
        self.pub_mr = self.create_publisher(Float32, self.get_parameter('motor_right_topic').value, 20)
        self.pub_g2 = self.create_publisher(Float32, self.get_parameter('gimbal2_topic').value, 20)
        self.pub_g3 = self.create_publisher(Float32, self.get_parameter('gimbal3_topic').value, 20)
        self.pub_sf = self.create_publisher(Float32, self.get_parameter('susp_front_topic').value, 20)
        self.pub_sb = self.create_publisher(Float32, self.get_parameter('susp_back_topic').value, 20)

        self.last_stamp = None
        self.sub = self.create_subscription(Joy, joy_topic, self.on_joy, 50)

        self.get_logger().info(
            "TeleopPS5Node running. IMPORTANT: set deadman_button/l1_button/r1_button indices if needed."
        )

    def axis(self, msg: Joy, idx: int, default: float = 0.0) -> float:
        return msg.axes[idx] if idx < len(msg.axes) else default

    def button(self, msg: Joy, idx: int) -> int:
        return msg.buttons[idx] if idx < len(msg.buttons) else 0

    def trigger_pressed_amount(self, axis_val: float) -> float:
        # Linux common: unpressed=+1.0, pressed=-1.0 -> map to 0..1
        return clamp((1.0 - axis_val) / 2.0, 0.0, 1.0)

    def on_joy(self, msg: Joy):
        # Deadman (X)
        enabled = self.button(msg, self.deadman_button) == 1

        # Time delta for gimbal integration
        if self.last_stamp is None:
            self.last_stamp = msg.header.stamp
            dt = 0.0
        else:
            now = msg.header.stamp
            dt = (now.sec - self.last_stamp.sec) + (now.nanosec - self.last_stamp.nanosec) * 1e-9
            dt = clamp(dt, 0.0, 0.1)  # avoid crazy jumps
            self.last_stamp = now

        # ----- Driving -----
        throttle_raw = self.axis(msg, self.throttle_axis, 0.0)
        turn_raw = self.axis(msg, self.turn_axis, 0.0)

        # Invert Y so pushing stick forward => +throttle
        throttle = (-throttle_raw) * self.throttle_gain
        turn = (turn_raw) * self.turn_gain

        if not enabled:
            left = 0.0
            right = 0.0
        else:
            left = clamp(throttle + turn, -1.0, 1.0) * self.max_cmd
            right = clamp(throttle - turn, -1.0, 1.0) * self.max_cmd

        # ----- Gimbal (integrate right stick into angles) -----
        if enabled and dt > 0.0:
            g2_axis = self.axis(msg, self.gimbal2_axis, 0.0)
            g3_axis = self.axis(msg, self.gimbal3_axis, 0.0)

            # You may want to invert one axis depending on feel
            self.g2 += g2_axis * self.gimbal_rate * dt
            self.g3 += (-g3_axis) * self.gimbal_rate * dt  # invert so up => increase

            self.g2 = clamp(self.g2, self.gimbal_min, self.gimbal_max)
            self.g3 = clamp(self.g3, self.gimbal_min, self.gimbal_max)

        # ----- Suspension speeds -----
        # Front: L1 up, L2 down
        front = 0.0
        if enabled:
            l1 = self.button(msg, self.l1_button) == 1
            l2 = False
            if self.l2_is_axis:
                l2 = self.trigger_pressed_amount(self.axis(msg, self.l2_axis, 1.0)) > 0.2
            else:
                l2 = self.button(msg, self.l2_button) == 1

            if l1 and not l2:
                front = +self.susp_speed
            elif l2 and not l1:
                front = -self.susp_speed

        # Back: R1 up, R2 down
        back = 0.0
        if enabled:
            r1 = self.button(msg, self.r1_button) == 1
            r2 = False
            if self.r2_is_axis:
                r2 = self.trigger_pressed_amount(self.axis(msg, self.r2_axis, 1.0)) > 0.2
            else:
                r2 = self.button(msg, self.r2_button) == 1

            if r1 and not r2:
                back = +self.susp_speed
            elif r2 and not r1:
                back = -self.susp_speed

        # Publish
        def pub(p, val):
            m = Float32(); m.data = float(val); p.publish(m)

        pub(self.pub_ml, left)
        pub(self.pub_mr, right)
        pub(self.pub_g2, self.g2)
        pub(self.pub_g3, self.g3)
        pub(self.pub_sf, front)
        pub(self.pub_sb, back)

def main():
    rclpy.init()
    node = TeleopPS5Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
