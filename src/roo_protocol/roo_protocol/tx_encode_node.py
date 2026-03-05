#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

def frame(dev_id: int, payload: str) -> str:
    return f"${dev_id}~{payload}`"

class TxEncodeNode(Node):
    def __init__(self):
        super().__init__('tx_encode_node')

        self.declare_parameter('tx_buffer_topic', '/roo/tx_buffer')
        tx_topic = self.get_parameter('tx_buffer_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(String, tx_topic, 50)

        # Subscriptions (commands)
        self.create_subscription(Float32, '/roo/cmd/gimbal_servo_1', self.cb_g1, 20)
        self.create_subscription(Float32, '/roo/cmd/gimbal_servo_2', self.cb_g2, 20)
        self.create_subscription(Float32, '/roo/cmd/gimbal_servo_3', self.cb_g3, 20)

        self.create_subscription(Float32, '/roo/cmd/suspension_front', self.cb_sf, 20)
        self.create_subscription(Float32, '/roo/cmd/suspension_back',  self.cb_sb, 20)

        self.create_subscription(Float32, '/roo/cmd/motor_left',  self.cb_ml, 20)
        self.create_subscription(Float32, '/roo/cmd/motor_right', self.cb_mr, 20)

        self.get_logger().info(f"TxEncodeNode publishing frames to {tx_topic}")

    def publish_frame(self, dev_id: int, payload: str):
        msg = String()
        msg.data = frame(dev_id, payload)
        self.pub.publish(msg)

    # Servo angles: send as int degrees 0-180 (you can change formatting later)
    def cb_g1(self, msg: Float32): self.publish_frame(1, str(int(msg.data)))
    def cb_g2(self, msg: Float32): self.publish_frame(2, str(int(msg.data)))
    def cb_g3(self, msg: Float32): self.publish_frame(3, str(int(msg.data)))

    # Suspension + motors: your README says -10..+10 (send float but compact)
    def cb_sf(self, msg: Float32): self.publish_frame(4, f"{msg.data:.3f}".rstrip('0').rstrip('.'))
    def cb_sb(self, msg: Float32): self.publish_frame(5, f"{msg.data:.3f}".rstrip('0').rstrip('.'))

    def cb_ml(self, msg: Float32): self.publish_frame(6, f"{msg.data:.3f}".rstrip('0').rstrip('.'))
    def cb_mr(self, msg: Float32): self.publish_frame(7, f"{msg.data:.3f}".rstrip('0').rstrip('.'))

def main():
    rclpy.init()
    node = TxEncodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
