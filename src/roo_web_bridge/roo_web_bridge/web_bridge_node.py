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
    g2: float = 0.0
    g3: float = 0.0
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
        self.pub_cmd = self.create_publisher(String, '/roo/cmd', 10)

        # Publish control source based on whether any web clients are connected
        self._source_timer = self.create_timer(0.2, self._publish_source_state)

        # Telemetry subscriptions (adjust names if yours differ)
        self.sub_v = self.create_subscription(Float32, '/roo/battery/voltage', self._on_v, 10)
        self.sub_i = self.create_subscription(Float32, '/roo/battery/current', self._on_i, 10)
        self.sub_e = self.create_subscription(Float32, '/roo/battery/energy', self._on_e, 10)
        self.sub_pitch = self.create_subscription(Float32, '/roo/telemetry/pitch_deg', self._on_pitch, 10)
        self.sub_roll  = self.create_subscription(Float32, '/roo/telemetry/roll_deg', self._on_roll, 10)
        self.sub_gimbal_enc1 = self.create_subscription(Float32, '/roo/gimbal/enc1_angle', self._on_gimbal_enc1, 10)
        self.sub_gimbal_enc2 = self.create_subscription(Float32, '/roo/gimbal/enc2_angle', self._on_gimbal_enc2, 10)

        self.cmd = CmdState()
        self.telem = {
            "voltage_v": None,
            "current_a": None,
            "energy_mwh": None,
            "pitch_deg": None,
            "roll_deg": None,
            "enc1_angle": None,
            "enc2_angle": None,
            "battery_pct": None,
        }

        self._ws_clients = set()
        self._lock = threading.Lock()

        # Start websocket server in background thread
        self._thread = threading.Thread(target=self._run_ws_thread, daemon=True)
        self._thread.start()

        self.get_logger().info(f'roo_web_bridge listening on ws://{self.ws_host}:{self.ws_port}')

    def _on_v(self, msg: Float32):
        v = float(msg.data)
        self.telem["voltage_v"] = v
        
        # Auto-detect cell count and calculate rough LiPo percentage
        if v > 13.5:      # 4S LiPo (13.2V empty - 16.8V full)
            pct = max(0.0, min(100.0, (v - 13.2) / (16.8 - 13.2) * 100))
        elif v > 9.5:     # 3S LiPo (9.9V empty - 12.6V full)
            pct = max(0.0, min(100.0, (v - 9.9) / (12.6 - 9.9) * 100))
        elif v > 6.5:     # 2S LiPo (6.6V empty - 8.4V full)
            pct = max(0.0, min(100.0, (v - 6.6) / (8.4 - 6.6) * 100))
        else:
            pct = 0.0
        self.telem["battery_pct"] = round(pct, 1)

    def _on_i(self, msg: Float32): self.telem["current_a"] = float(msg.data)
    def _on_e(self, msg: Float32): self.telem["energy_mwh"] = float(msg.data)
    def _on_pitch(self, msg: Float32): self.telem["pitch_deg"] = float(msg.data)
    def _on_roll(self, msg: Float32): self.telem["roll_deg"] = float(msg.data)
    def _on_gimbal_enc1(self, msg: Float32): self.telem["enc1_angle"] = float(msg.data)
    def _on_gimbal_enc2(self, msg: Float32): self.telem["enc2_angle"] = float(msg.data)

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
                    self.cmd.g2 = 0.0
                    self.cmd.g3 = 0.0
                elif t == "motor":
                    self.cmd.motor_left = float(m.get("left", 0.0))
                    self.cmd.motor_right = float(m.get("right", 0.0))
                elif t == "gimbal":
                    self.cmd.g2 = float(m.get("g2", 0.0))
                    self.cmd.g3 = float(m.get("g3", 0.0))
                elif t == "susp":
                    self.cmd.susp_front = float(m.get("front", 0.0))
                    self.cmd.susp_back = float(m.get("back", 0.0))
                elif t == "command":
                    msg = String()
                    msg.data = f"{m.get('id')},{m.get('value')}"
                    self.pub_cmd.publish(msg)
                elif t == "heartbeat":
                    pass

                # publish after each message
                self._publish_cmds()

                # send telemetry snapshot back quickly
                await self._broadcast_telem()

        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info('Browser disconnected gracefully.')
        except ConnectionResetError:
            self.get_logger().warn('Browser connection reset (page refreshed).')
        except Exception as e:
            self.get_logger().error(f'WebSocket error: {e}')
        
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
