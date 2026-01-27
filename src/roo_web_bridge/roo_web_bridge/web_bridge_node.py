#!/usr/bin/env python3
import json
import asyncio
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

import websockets


@dataclass
class CmdState:
    motor_left: float = 0.0
    motor_right: float = 0.0
    g2: float = 90.0
    g3: float = 90.0
    susp_front: float = 0.0
    susp_back: float = 0.0
    control_source: str = "web"


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge')

        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)

        self.ws_host = self.get_parameter('ws_host').get_parameter_value().string_value
        self.ws_port = int(self.get_parameter('ws_port').get_parameter_value().integer_value)

        # Publishers (match your mux inputs)
        self.pub_motor_left  = self.create_publisher(Float32, '/roo/web_cmd/motor_left', 10)
        self.pub_motor_right = self.create_publisher(Float32, '/roo/web_cmd/motor_right', 10)
        self.pub_g2 = self.create_publisher(Float32, '/roo/web_cmd/gimbal_servo_2', 10)
        self.pub_g3 = self.create_publisher(Float32, '/roo/web_cmd/gimbal_servo_3', 10)
        self.pub_sf = self.create_publisher(Float32, '/roo/web_cmd/suspension_front', 10)
        self.pub_sb = self.create_publisher(Float32, '/roo/web_cmd/suspension_back', 10)
        self.pub_src = self.create_publisher(String, '/roo/control_source', 10)

        # Publish control source based on whether any web clients are connected
        self._source_timer = self.create_timer(0.2, self._publish_source_state)

        # Telemetry subscriptions (adjust names if yours differ)
        self.sub_v = self.create_subscription(Float32, '/roo/telemetry/voltage_v', self._on_v, 10)
        self.sub_i = self.create_subscription(Float32, '/roo/telemetry/current_a', self._on_i, 10)
        self.sub_pitch = self.create_subscription(Float32, '/roo/telemetry/pitch_deg', self._on_pitch, 10)
        self.sub_roll  = self.create_subscription(Float32, '/roo/telemetry/roll_deg', self._on_roll, 10)

        self.cmd = CmdState()
        self.telem = {
            "voltage_v": None,
            "current_a": None,
            "pitch_deg": None,
            "roll_deg": None,
        }

        self._ws_clients = set()
        self._lock = threading.Lock()

        # Start websocket server in background thread
        self._thread = threading.Thread(target=self._run_ws_thread, daemon=True)
        self._thread.start()

        self.get_logger().info(f'roo_web_bridge listening on ws://{self.ws_host}:{self.ws_port}')

    def _on_v(self, msg: Float32): self.telem["voltage_v"] = float(msg.data)
    def _on_i(self, msg: Float32): self.telem["current_a"] = float(msg.data)
    def _on_pitch(self, msg: Float32): self.telem["pitch_deg"] = float(msg.data)
    def _on_roll(self, msg: Float32): self.telem["roll_deg"] = float(msg.data)

    def _publish_source_state(self):
        # If any WS clients connected -> web. Otherwise -> joy.
        src = String()
        src.data = 'web' if len(self._ws_clients) > 0 else 'joy'
        self.pub_src.publish(src)


    def _publish_cmds(self):
        # publish control source (web) so mux can select
  #      src = String()
 #       src.data = self.cmd.control_source
#        self.pub_src.publish(src)

        ml = Float32(); ml.data = float(self.cmd.motor_left)
        mr = Float32(); mr.data = float(self.cmd.motor_right)
        g2 = Float32(); g2.data = float(self.cmd.g2)
        g3 = Float32(); g3.data = float(self.cmd.g3)
        sf = Float32(); sf.data = float(self.cmd.susp_front)
        sb = Float32(); sb.data = float(self.cmd.susp_back)

        self.pub_motor_left.publish(ml)
        self.pub_motor_right.publish(mr)
        self.pub_g2.publish(g2)
        self.pub_g3.publish(g3)
        self.pub_sf.publish(sf)
        self.pub_sb.publish(sb)

    async def _broadcast_telem(self):
        payload = {"type": "telemetry", **self.telem}
        msg = json.dumps(payload)
        dead = []
        for ws in list(self._ws_clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._ws_clients.discard(ws)

    async def _ws_handler(self, websocket):
        with self._lock:
            self._ws_clients.add(websocket)

        try:
            async for raw in websocket:
                try:
                    m = json.loads(raw)
                except Exception:
                    continue

                t = m.get("type")
                if t == "control_source":
                    self.cmd.control_source = str(m.get("value", "web"))
                elif t == "stop":
                    self.cmd.motor_left = 0.0
                    self.cmd.motor_right = 0.0
                    self.cmd.susp_front = 0.0
                    self.cmd.susp_back = 0.0
                elif t == "motor":
                    self.cmd.motor_left = float(m.get("left", 0.0))
                    self.cmd.motor_right = float(m.get("right", 0.0))
                elif t == "gimbal":
                    self.cmd.g2 = float(m.get("g2", 90.0))
                    self.cmd.g3 = float(m.get("g3", 90.0))
                elif t == "susp":
                    self.cmd.susp_front = float(m.get("front", 0.0))
                    self.cmd.susp_back = float(m.get("back", 0.0))
                elif t == "heartbeat":
                    pass

                # publish after each message
                self._publish_cmds()

                # send telemetry snapshot back quickly
                await self._broadcast_telem()

        finally:
            with self._lock:
                self._ws_clients.discard(websocket)

    def _run_ws_thread(self):
        async def runner():
            async with websockets.serve(self._ws_handler, self.ws_host, self.ws_port):
                while rclpy.ok():
                    # periodic telemetry broadcast
                    await self._broadcast_telem()
                    await asyncio.sleep(0.2)

        asyncio.run(runner())


def main():
    rclpy.init()
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
