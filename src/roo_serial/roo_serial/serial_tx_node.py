#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import threading

class SerialBridgeNode(Node):
    """
    A single node to handle bidirectional communication over a serial port.
    It subscribes to a topic for outgoing data (TX) and publishes
    data it receives from the serial port (RX) to another topic.
    """
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/roo_esp32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('tx_topic', '/roo/tx_buffer')
        self.declare_parameter('rx_topic', '/roo/rx_buffer')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        tx_topic = self.get_parameter('tx_topic').get_parameter_value().string_value
        rx_topic = self.get_parameter('rx_topic').get_parameter_value().string_value

        self.tx_sub = self.create_subscription(String, tx_topic, self.on_tx_msg, 50)
        self.rx_pub = self.create_publisher(String, rx_topic, 50)

        self.ser = None
        self.read_thread = None
        self._running = True
        
        self.get_logger().info(f"SerialBridgeNode starting: port={self.port} baud={self.baud}")

        # Connect at startup and start the reader thread
        self.connect_and_start_reader()

    def connect_and_start_reader(self):
        if self.ser and self.ser.is_open:
            return True
        try:
            self.get_logger().info(f"Attempting to connect to {self.port}...")
            # Use a timeout for the read loop
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info("Serial connected.")
            
            # Start the reading thread
            self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
            self.read_thread.start()
            return True
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"Serial connect failed: {e}")
            return False

    def on_tx_msg(self, msg: String):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("TX: Serial not connected. Attempting reconnect.")
            if not self.connect_and_start_reader():
                self.get_logger().warn("TX: Reconnect failed. Dropping message.")
                return

        try:
            out = msg.data.encode('utf-8')
            self.ser.write(out)
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}. The read thread will handle any required reconnection.")
            # Do not close the port or set self.ser to None here.
            # Let the read_loop detect the disconnected port via its own exception handling.

    def read_loop(self):
        self.get_logger().info("Serial read thread started.")
        buffer = b''
        while rclpy.ok() and self._running:
            try:
                if not self.ser or not self.ser.is_open:
                    self.get_logger().error("Read loop: Serial port closed. Exiting thread.")
                    break
                
                # read() will block up to the timeout and return any available data
                data = self.ser.read(4096)
                if not data:
                    continue

                buffer += data
                
                # Process buffer for complete '$...`' frames
                while True:
                    start_index = buffer.find(b'$')
                    if start_index == -1:
                        # No frame start found. To prevent unbounded buffer growth,
                        # we can discard some data, but it's safer to just wait.
                        # If your device sends non-frame data, you might want to trim the buffer here.
                        break # Wait for more data

                    # Discard any garbage before the start of a frame
                    if start_index > 0:
                        buffer = buffer[start_index:]
                    
                    # Now that buffer starts with '$', look for the end '`'
                    end_index = buffer.find(b'`')
                    if end_index == -1:
                        # Have a start but not an end yet. Wait for more data.
                        break

                    # We have a full frame
                    frame_data = buffer[:end_index + 1]
                    buffer = buffer[end_index + 1:] # Keep remainder for next loop

                    ros_msg = String()
                    ros_msg.data = frame_data.decode('utf-8', 'ignore')
                    self.rx_pub.publish(ros_msg)

            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}. Exiting thread.")
                self.ser = None # Port is gone
                break
            except Exception as e:
                self.get_logger().error(f"Unhandled error in read_loop: {e}")
        self.get_logger().info("Serial read thread finished.")

    def destroy_node(self):
        self._running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = SerialBridgeNode()
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
