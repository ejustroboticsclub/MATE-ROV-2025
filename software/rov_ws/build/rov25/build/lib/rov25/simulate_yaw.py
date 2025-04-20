import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf_transformations


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion (roll, pitch, yaw in radians)."""
    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class IMUInputPublisher(Node):
    def __init__(self):
        super().__init__('imu_input_publisher')
        self.publisher_ = self.create_publisher(Imu, '/ROV/imu', 10)
        self.get_logger().info("IMU Publisher node has started.")

    def publish_angle(self, angle_deg: float):
        angle_rad = math.radians(angle_deg)
        quat = euler_to_quaternion(0.0, 0.0, angle_rad)  # yaw only

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"
        msg.orientation = quat

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published quaternion for angle {angle_deg:.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = IMUInputPublisher()

    try:
        while rclpy.ok():
            try:
                angle_input = input("Enter yaw angle in degrees (0–360): ")
                angle = float(angle_input)
                if 0.0 <= angle <= 360.0:
                    node.publish_angle(angle)
                else:
                    print("Please enter an angle between 0 and 360.")
            except ValueError:
                print("Invalid input. Please enter a numeric angle.")
    except KeyboardInterrupt:
        print("\nShutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
