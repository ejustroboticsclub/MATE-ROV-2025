#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float64MultiArray, String, Bool, Int32, Int8, Int32MultiArray

# CAN IDs
THRUSTERS_ID = 0x100
GRIPPERS_ID_L = 0x101
GRIPPERS_ID_R = 0x102
GRIPPERS_ID_C = 0x103  # Center gripper CAN ID
PUMP_ID = 0x300
QUAT_COMMAND_ID = 0x200

# IMU IDs (not used in this script but kept for completeness)
IMU1_ID = 0x201
IMU2_ID = 0x202
IMU3_ID = 0x203
IMU4_ID = 0x204

THRUSTER_COUNT = 7

class CANBridge(Node):
    def __init__(self):
        super().__init__('can_ros2_bridge')

        # Init CAN interface
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("CAN interface initialized on can0")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN interface: {e}")
            raise

        # Init ROS subscribers
        self.create_subscription(Int32MultiArray, '/ROV/thrusters', self.thruster_callback, 10)
        self.create_subscription(Bool, '/ROV/gripper_l', self.gripper_l_callback, 10)
        self.create_subscription(Bool, '/ROV/gripper_r', self.gripper_r_callback, 10)
        self.create_subscription(Int8, '/ROV/pump', self.pump_callback, 10)
        self.create_subscription(String, '/Commands', self.command_callback, 10)

        # Init state
        self.center_gripper_state = 0

        self.get_logger().info("CAN-ROS2 Bridge Node Started.")

    def send_msg(self, can_id, data):
        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=bytes(data)
        )
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

        thruster_bytes = []
        for pwm in vals:
            p = max(1000, min(2000, pwm))
            thruster_bytes.append(p // 10)  # Convert 1000–2000 to 100–200

        if self.send_msg(THRUSTERS_ID, thruster_bytes):
            self.get_logger().info("Sent Thruster PWM values.")
        else:
            self.get_logger().warn("Failed to send Thruster values.")

    def gripper_l_callback(self, msg):
        state = 1 if msg.data else 0
        if self.send_msg(GRIPPERS_ID_L, [state]):
            self.get_logger().info(f"Sent Left Gripper state: {state}")
        else:
            self.get_logger().warn("Failed to send Left Gripper state.")

    def gripper_r_callback(self, msg):
        state = 1 if msg.data else 0
        if self.send_msg(GRIPPERS_ID_R, [state]):
            self.get_logger().info(f"Sent Right Gripper state: {state}")
        else:
            self.get_logger().warn("Failed to send Right Gripper state.")

    def pump_callback(self, msg):
        val = msg.data

        if val == 4:

            self.center_gripper_state ^= 1
            if self.send_msg(GRIPPERS_ID_C, [self.center_gripper_state]):
                state_str = "OPEN" if self.center_gripper_state else "CLOSE"
                self.get_logger().info(f"Toggled Center Gripper: {state_str}")
            else:
                self.get_logger().warn("Failed to send Center Gripper toggle.")
        else:

            val_masked = val & 0x03
            if self.send_msg(PUMP_ID, [val_masked]):
                state = {0: "Stop", 1: "Clockwise", 2: "Counter-Clockwise"}.get(val_masked, "Unknown")
                self.get_logger().info(f"Sent Pump Direction: {state}")
            else:
                self.get_logger().warn("Failed to send pump direction.")

    def command_callback(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "CALIBRATE":
            if self.send_msg(QUAT_COMMAND_ID, [0x01]):
                self.get_logger().info("Sent CALIBRATE command.")
            else:
                self.get_logger().warn("Failed to send CALIBRATE.")
        elif cmd == "RESET":
            if self.send_msg(QUAT_COMMAND_ID, [0x02]):
                self.get_logger().info("Sent RESET command.")
            else:
                self.get_logger().warn("Failed to send RESET.")
        else:
            self.get_logger().warn(f"Unknown command received: '{cmd}'")

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
