import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import can
import struct

# Define CAN IDs
IMU1_ID = 0x201  # Quaternion x
IMU2_ID = 0x202  # Quaternion y
IMU3_ID = 0x203  # Quaternion z
IMU4_ID = 0x204  # Quaternion w
DEPTH_ID = 0x205 # Depth
Gx_ID    = 0x206 # Angular velocity x
Gy_ID    = 0x207 # Angular velocity y
Gz_ID    = 0x208 # Angular velocity z

class CANReceiverNode(Node):
    def __init__(self):
        super().__init__('can_receiver_node')

        # Publishers
        self.imu_publisher = self.create_publisher(Imu, 'ROV/IMU', 10)
        self.depth_publisher = self.create_publisher(Float32, 'ROV/Depth', 10)

        # Set up CAN bus
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        # Initialize data placeholders
        self.qx = self.qy = self.qz = self.qw = None
        self.gx = self.gy = self.gz = None
        self.depth = None

        # Start periodic polling
        self.create_timer(0.005, self.listen_can_messages)

    def listen_can_messages(self):
        message = self.bus.recv(timeout=0.005)
        if message is None:
            return

        data_float = self.extract_float_from_can_data(message.data)
        if data_float is None:
            return

        msg_id = message.arbitration_id

        if msg_id == IMU1_ID:
            self.qx = data_float
        elif msg_id == IMU2_ID:
            self.qy = data_float
        elif msg_id == IMU3_ID:
            self.qz = data_float
        elif msg_id == IMU4_ID:
            self.qw = data_float
        elif msg_id == Gx_ID:
            self.gx = data_float
        elif msg_id == Gy_ID:
            self.gy = data_float
        elif msg_id == Gz_ID:
            self.gz = data_float
        elif msg_id == DEPTH_ID:
            self.depth = data_float
            self.publish_depth()

        # Publish IMU when all components are received
        if all(v is not None for v in [self.qx, self.qy, self.qz, self.qw, self.gx, self.gy, self.gz]):
            self.publish_imu()

    def extract_float_from_can_data(self, data):
        if len(data) != 4:
            pass
            return None
        return struct.unpack('<f', data)[0]

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # Quaternion
        imu_msg.orientation.x = self.qx
        imu_msg.orientation.y = self.qy
        imu_msg.orientation.z = self.qz
        imu_msg.orientation.w = self.qw

        # Angular velocity (gyro)
        imu_msg.angular_velocity.x = self.gx
        imu_msg.angular_velocity.y = self.gy
        imu_msg.angular_velocity.z = self.gz

        # Zero linear acceleration for now
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        self.imu_publisher.publish(imu_msg)
        self.get_logger().info("Published IMU data")

        # Clear values after publishing to avoid duplication
        self.qx = self.qy = self.qz = self.qw = None
        self.gx = self.gy = self.gz = None

    def publish_depth(self):
        if self.depth is not None:
            depth_msg = Float32()
            depth_msg.data = self.depth
            self.depth_publisher.publish(depth_msg)
            self.get_logger().info(f"Published Depth: {self.depth:.2f}")
            self.depth = None

def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

