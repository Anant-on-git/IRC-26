import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from serial_bridge_msgs.msg import ArmPos

import serial
import threading
import json
from serial.serialutil import SerialException


# ---------- Serial framing ----------
STX = b'\x02'  # Start of Text
ETX = b'\x03'  # End of Text


# ---------- CRC-8 (poly 0x07) ----------
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Serial config (hardcoded for now)
        self.port = '/dev/ttyACM0'
        self.baud = 115200

        self.get_logger().info("Starting Serial Bridge Node")
        self.get_logger().info(f"Opening serial port {self.port} @ {self.baud}")

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info("Serial connection established")
        except SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Cached latest values
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.joint1 = 0.0
        self.joint2 = 0.0

        self.msg_id = 0
        self.lock = threading.Lock()

        # Subscribers
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            10
        )

        self.create_subscription(
            ArmPos,
            'arm_pos',
            self.arm_pos_cb,
            10
        )

        self.get_logger().info("Subscribed to /cmd_vel and /arm_pos")

    # ---------- Callbacks ----------

    def cmd_vel_cb(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        self.get_logger().info(
            f"cmd_vel | linear.x={self.linear_x:.3f}, angular.z={self.angular_z:.3f}"
        )

        self.send_packet()

    def arm_pos_cb(self, msg: ArmPos):
        self.joint1 = msg.joint1_pos
        self.joint2 = msg.joint2_pos

        self.get_logger().info(
            f"arm_pos | joint1={self.joint1:.3f}, joint2={self.joint2:.3f}"
        )

        self.send_packet()

    # ---------- JSON Packet Sender ----------

    def send_packet(self):
        with self.lock:
            self.msg_id += 1

            packet = {
                "v": 1,
                "type": "cmd",
                "id": self.msg_id,
                "topic": "control",
                "data": {
                    "cmd_vel": {
                        "linear": {
                            "x": self.linear_x
                        },
                        "angular": {
                            "z": self.angular_z
                        }
                    },
                    "arm_pos": {
                        "joint1_pos": self.joint1,
                        "joint2_pos": self.joint2
                    }
                }
            }

            # Serialize without CRC
            json_no_crc = json.dumps(
                packet,
                separators=(',', ':')
            ).encode()

            # Compute CRC
            packet["crc"] = crc8(json_no_crc)

            # Final JSON
            final_json = json.dumps(
                packet,
                separators=(',', ':')
            ).encode()

            try:
                self.ser.write(STX + final_json + ETX)
                self.get_logger().info(
                    f"Serial JSON sent | id={self.msg_id}"
                )
            except SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")


def main():
    rclpy.init()
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Serial Bridge")
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("Serial port closed")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
