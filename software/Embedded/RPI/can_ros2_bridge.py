#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
from threading import Thread
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Int32MultiArray, String, Bool, Int8

# CAN IDs
THRUSTERS_ID = 0x100
GRIPPERS_ID_L = 0x101
GRIPPERS_ID_R = 0x102
GRIPPERS_ID_C = 0x103
PUMP_ID = 0x300
QUAT_COMMAND_ID = 0x200

# Custom incoming CAN IDs
CURRENTS_IDS = [0x310, 0x312, 0x315, 0x317]
VOLTAGES_IDS = [0x110, 0x112, 0x115]
HEARTBEAT_IDS = [0x320, 0x209, 0x120]

THRUSTER_COUNT = 7

class CANBridge(Node):
    def __init__(self):
        super().__init__('can_ros2_bridge')

        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("CAN interface initialized on can0")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN interface: {e}")
            raise

        # ROS2 Publishers
        self.voltage_pub = self.create_publisher(Float32MultiArray, '/ROV/voltage', 10)
        self.current_pub = self.create_publisher(Float32MultiArray, '/ROV/current', 10)
        self.indicator_pub = self.create_publisher(Int8MultiArray, '/ROV/indicators', 10)
        self.jelly_pub = self.create_publisher(Bool, '/ROV/jelly_indicators', 10)
        self.jelly_pub.publish(Bool(data=False)) 
        # ROS2 Subscribers
        self.create_subscription(Int32MultiArray, '/ROV/thrusters', self.thruster_callback, 10)
        self.create_subscription(Bool, '/ROV/gripper_l', self.gripper_l_callback, 10)
        self.create_subscription(Bool, '/ROV/gripper_r', self.gripper_r_callback, 10)
        self.create_subscription(Int8, '/ROV/pump', self.pump_callback, 10)
        self.create_subscription(String, '/Commands', self.command_callback, 10)

        self.center_gripper_state = 0
        self.jelly_indicator_state = False

        # Start CAN listener
        self.listener_thread = Thread(target=self.listen_to_can)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        self.get_logger().info("CAN-ROS2 Bridge Node Started.")

    def send_msg(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, is_extended_id=False, data=bytes(data))
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            self.get_logger().error(f"CAN send error: {e}")
            return False

    def thruster_callback(self, msg):
        vals = msg.data
        if len(vals) != THRUSTER_COUNT:
            self.get_logger().warn("Thruster array must contain 7 values.")
            return
        thruster_bytes = [max(100, min(200, pwm // 10)) for pwm in vals]
        self.send_msg(THRUSTERS_ID, thruster_bytes)

    def gripper_l_callback(self, msg):
        self.send_msg(GRIPPERS_ID_L, [1 if msg.data else 0])

    def gripper_r_callback(self, msg):
        self.send_msg(GRIPPERS_ID_R, [1 if msg.data else 0])

    def pump_callback(self, msg):
        val = msg.data
        if val == 4:
            # Toggle center gripper state
            self.center_gripper_state ^= 1
            self.send_msg(GRIPPERS_ID_C, [self.center_gripper_state])

            # Toggle jelly indicator state and publish
            self.jelly_indicator_state = not self.jelly_indicator_state
            self.jelly_pub.publish(Bool(data=self.jelly_indicator_state))
        else:
            self.send_msg(PUMP_ID, [val & 0x03])

    def command_callback(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "CALIBRATE":
            self.send_msg(QUAT_COMMAND_ID, [0x01])
        elif cmd == "RESET":
            self.send_msg(QUAT_COMMAND_ID, [0x02])
        else:
            self.get_logger().warn(f"Unknown command received: '{cmd}'")

    def listen_to_can(self):
        while rclpy.ok():
            msg = self.bus.recv()
            if msg is None:
                continue

            can_id = msg.arbitration_id
            data = msg.data

            if can_id in VOLTAGES_IDS:
                voltages = []
                for i in range(0, len(data), 2):
                    if i + 1 < len(data):
                        raw = (data[i] << 8) | data[i + 1]
                        voltage = raw / 100.0
                        voltages.append(voltage)
                self.voltage_pub.publish(Float32MultiArray(data=voltages))

            elif can_id in CURRENTS_IDS:
                currents = []
                for i in range(0, len(data), 2):
                    if i + 1 < len(data):
                        raw = (data[i] << 8) | data[i + 1]
                        current = raw / 100.0
                        currents.append(current)
                self.current_pub.publish(Float32MultiArray(data=currents))

            elif can_id in HEARTBEAT_IDS:
                indicators_array = Int8MultiArray()
                indicators_array.data = list(data)
                self.indicator_pub.publish(indicators_array)

def main(args=None):
    rclpy.init(args=args)
    node = CANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CAN-ROS2 Bridge.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
