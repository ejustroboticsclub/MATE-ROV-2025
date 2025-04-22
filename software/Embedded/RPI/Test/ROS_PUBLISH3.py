import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, Int32, String


class ManualPublisher(Node):
    def __init__(self):
        super().__init__('manual_publisher')

        # Create publishers for thrusters, grippers, pump, and commands
        self.thruster_pub = self.create_publisher(Int32MultiArray, '/ROV/thrusters', 10)
        self.gripper_l_pub = self.create_publisher(Bool, '/ROV/gripper_l', 10)
        self.gripper_r_pub = self.create_publisher(Bool, '/ROV/gripper_r', 10)
        self.pump_pub = self.create_publisher(Int32, '/Pump', 10)
        self.command_pub = self.create_publisher(String, '/Commands', 10)

        self.get_logger().info("Manual publisher ready. Enter commands:")
        self.run_input_loop()

    def run_input_loop(self):
        print("""
Available Commands:
- PWM <7 values>         → e.g. PWM 1500 1500 1500 1500 1500 1500 1500
- GRIPPER_L <1 value>    → e.g. GRIPPER_L 1
- GRIPPER_R <1 value>    → e.g. GRIPPER_R 0
- PUMP <1 value>         → e.g. PUMP 2
- CALIBRATE / RESET      → send command to /commands
        """)

        while rclpy.ok():
            try:
                line = input(">>> ").strip()
                if not line:
                    continue

                parts = line.split()
                cmd_type = parts[0].upper()

                if cmd_type in ["CALIBRATE", "RESET"]:
                    self.send_command(cmd_type)
                    continue

                if cmd_type == "PWM":
                    if len(parts) != 8:
                        print("PWM command needs exactly 7 values.")
                        continue
                    try:
                        pwms = list(map(int, parts[1:]))
                        self.send_thrusters(pwms)
                    except ValueError:
                        print("Invalid PWM values.")
                        continue

                elif cmd_type == "GRIPPER_L":
                    if len(parts) != 2:
                        print("GRIPPER_L command needs exactly 1 value.")
                        continue
                    try:
                        gripper_l_action = bool(int(parts[1]))  # Convert 0 or 1 to bool
                        self.send_gripper_l(gripper_l_action)
                    except ValueError:
                        print("Invalid gripper value. Use 0 or 1.")
                        continue

                elif cmd_type == "GRIPPER_R":
                    if len(parts) != 2:
                        print("GRIPPER_R command needs exactly 1 value.")
                        continue
                    try:
                        gripper_r_action = bool(int(parts[1]))  # Convert 0 or 1 to bool
                        self.send_gripper_r(gripper_r_action)
                    except ValueError:
                        print("Invalid gripper value. Use 0 or 1.")
                        continue

                elif cmd_type == "PUMP":
                    if len(parts) != 2:
                        print("PUMP command needs exactly 1 value.")
                        continue
                    try:
                        pump_dir = int(parts[1])
                        self.send_pump(pump_dir)
                    except ValueError:
                        print("Invalid pump value.")
                        continue

                else:
                    print("Unknown command. Type PWM, GRIPPER_L, GRIPPER_R, PUMP, CALIBRATE or RESET.")

            except (EOFError, KeyboardInterrupt):
                print("\nShutting down manual publisher.")
                break

    def send_thrusters(self, pwms):
        msg = Int32MultiArray()
        msg.data = pwms
        self.thruster_pub.publish(msg)
        print("Published thruster PWMs:", pwms)

    def send_gripper_l(self, action):
        # Publish the left gripper status as a boolean (True/False)
        msg = Bool()
        msg.data = action
        self.gripper_l_pub.publish(msg)
        print(f"Published left gripper status: {action}")

    def send_gripper_r(self, action):
        # Publish the right gripper status as a boolean (True/False)
        msg = Bool()
        msg.data = action
        self.gripper_r_pub.publish(msg)
        print(f"Published right gripper status: {action}")

    def send_pump(self, pump_dir):
        msg = Int32()
        msg.data = pump_dir & 0x03  # Only keep the lower 2 bits (0-3)
        self.pump_pub.publish(msg)
        state = {0: "Stop", 1: "Clockwise", 2: "Counter-Clockwise"}.get(msg.data, "Unknown")
        print(f"Published pump direction: {state}")

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd.upper()
        self.command_pub.publish(msg)
        print(f"Published command: {cmd}")


def main(args=None):
    rclpy.init(args=args)
    node = ManualPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("ManualPublisher interrupted. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
