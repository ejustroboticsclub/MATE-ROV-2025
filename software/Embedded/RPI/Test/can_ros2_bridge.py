#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import struct
from std_msgs.msg import Int32MultiArray, Int32, String, Float64MultiArray, Bool

# CAN IDs for devices
THRUSTERS_ID = 0x100
GRIPPERS_ID_L = 0x101  # Left Gripper CAN ID
GRIPPERS_ID_R = 0x102  # Right Gripper CAN ID
PUMP_ID = 0x300
QUAT_COMMAND_ID = 0x200  # for calibration/reset commands

# IMU CAN IDs
IMU1_ID = 0x201
IMU2_ID = 0x202
IMU3_ID = 0x203
IMU4_ID = 0x204

THRUSTER_COUNT = 7

class CANBridge(Node):
    def __init__(self):
        super().__init__('can_ros2_bridge')

        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("CAN interface initialized on can0")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN interface: {e}")
            raise

        # ROS 2 subscriptions
        self.create_subscription(Int32MultiArray, '/ROV/thrusters', self.thruster_callback, 10)
        self.create_subscription(Bool, '/ROV/gripper_l', self.gripper_l_callback, 10)
        self.create_subscription(Bool, '/ROV/gripper_r', self.gripper_r_callback, 10)
        self.create_subscription(Int32, '/Pump', self.pump_callback, 10)
        self.create_subscription(String, '/Commands', self.command_callback, 10)

        # ROS 2 publishers for quaternion data
        self.imu_publishers = {
            IMU1_ID: self.create_publisher(Float64MultiArray, '/imu1_quat', 10),
            IMU2_ID: self.create_publisher(Float64MultiArray, '/imu2_quat', 10),
            IMU3_ID: self.create_publisher(Float64MultiArray, '/imu3_quat', 10),
            IMU4_ID: self.create_publisher(Float64MultiArray, '/imu4_quat', 10),
        }

        # Timer to continuously read CAN messages
        self.create_timer(0.01, self.read_can_messages)  # 100 Hz polling rate

        self.get_logger().info("CAN-ROS2 Bridge Node Started.")

    def send_msg(self, can_id, data):
        """Helper function to send CAN messages."""
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
        """Handle thruster PWM control."""
        vals = msg.data
        if len(vals) != THRUSTER_COUNT:
            self.get_logger().warn("Thruster array must contain 7 values.")
            return

        thruster_bytes = []
        for pwm in vals:
            p = max(1000, min(2000, pwm))  # Clamp PWM between 1000 and 2000
            thruster_bytes.append(p // 10)  # Map 1000–2000 PWM to 100–200

        if self.send_msg(THRUSTERS_ID, thruster_bytes):
            self.get_logger().info("Sent Thruster PWM values.")
        else:
            self.get_logger().warn("Failed to send Thruster values.")

    def gripper_l_callback(self, msg):
        """Handle left gripper state (open/close)."""
        gripper_l_state = 1 if msg.data else 0  # Convert bool to int (True -> 1, False -> 0)
        if self.send_msg(GRIPPERS_ID_L, [gripper_l_state]):  # Send to left gripper's CAN ID
            self.get_logger().info(f"Sent Left Gripper state: {gripper_l_state}")
        else:
            self.get_logger().warn("Failed to send Left Gripper state.")

    def gripper_r_callback(self, msg):
        """Handle right gripper state (open/close)."""
        gripper_r_state = 1 if msg.data else 0  # Convert bool to int (True -> 1, False -> 0)
        if self.send_msg(GRIPPERS_ID_R, [gripper_r_state]):  # Send to right gripper's CAN ID
            self.get_logger().info(f"Sent Right Gripper state: {gripper_r_state}")
        else:
            self.get_logger().warn("Failed to send Right Gripper state.")

    def pump_callback(self, msg):
        """Handle pump control (clockwise/counter-clockwise/stop)."""
        val = msg.data & 0x03  # Mask to only keep the lower 2 bits (stop/clockwise/counter-clockwise)
        if self.send_msg(PUMP_ID, [val]):
            state = {0: "Stop", 1: "Clockwise", 2: "Counter-Clockwise"}.get(val, "Unknown")
            self.get_logger().info(f"Sent Pump Direction: {state}")
        else:
            self.get_logger().warn("Failed to send pump direction.")

    def command_callback(self, msg):
        """Handle special commands like calibration and reset."""
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

    def read_can_messages(self):
        """Read incoming CAN messages and process them."""
        try:
            msg = self.bus.recv(timeout=0.001)  # Non-blocking with a small timeout
            if msg is None:
                return  # No message received

            can_id = msg.arbitration_id
            data = msg.data

            # Handle quaternion data for IMUs
            if can_id in self.imu_publishers and len(data) == 32:  # 4 doubles (32 bytes)
                quat = struct.unpack('<dddd', data)  # Unpack 4 doubles (little-endian)

                quat_msg = Float64MultiArray()
                quat_msg.data = list(quat)

                self.imu_publishers[can_id].publish(quat_msg)
                self.get_logger().debug(f"Published quaternion for IMU {hex(can_id)}: {quat}")

        except Exception as e:
            self.get_logger().warn(f"Error reading CAN: {e}")

def main(args=None):
    """Main entry point for the ROS 2 node."""
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
