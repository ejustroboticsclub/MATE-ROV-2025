import rclpy
from rclpy.node import Node
import random
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Bool, Int32MultiArray

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        # Create individual publisher attributes
        self.depth_pub = self.create_publisher(Float64, '/ROV/depth', 10)
        self.thrusters_pub = self.create_publisher(Int32MultiArray, 'ROV/thrusters', 10)
        self.imu_pub = self.create_publisher(Imu, 'ROV/imu', 10)
        self.gripper_r_pub = self.create_publisher(Bool, 'ROV/gripper_r', 10)
        self.gripper_l_pub = self.create_publisher(Bool, 'ROV/gripper_l', 10)
        self.float_pub = self.create_publisher(Float64, 'Float/depth', 10)

        # Set up timers
        self.create_timer(0.1, self.publish_thrusters)  # 10Hz for thrusters
        self.create_timer(0.5, self.publish_depth)      # 2Hz for depth
        self.create_timer(0.05, self.publish_imu)        # 20Hz for IMU
        self.create_timer(1.0, self.publish_grippers)    # 1Hz for grippers
        self.create_timer(0.8, self.publish_float)       # ~1.25Hz for float

    def publish_depth(self):
        msg = Float64()
        msg.data = round(random.uniform(0, 100), 2)
        self.depth_pub.publish(msg)
        self.get_logger().info(f"Published depth: {msg.data}m")

    def publish_thrusters(self):
        msg = Int32MultiArray()
        msg.data = [random.randint(1000, 2000) for _ in range(8)]
        self.thrusters_pub.publish(msg)
        self.get_logger().info("Published thrusters data")

    def publish_imu(self):
        msg = Imu()
        msg.linear_acceleration.x = round(random.uniform(-1.0, 1.0), 2)
        msg.linear_acceleration.y = round(random.uniform(-1.0, 1.0), 2)
        msg.linear_acceleration.z = round(random.uniform(8.0, 10.0), 2)
        msg.angular_velocity.x = round(random.uniform(-0.1, 0.1), 4)
        msg.angular_velocity.y = round(random.uniform(-0.1, 0.1), 4)
        msg.angular_velocity.z = round(random.uniform(-0.1, 0.1), 4)
        self.imu_pub.publish(msg)
        self.get_logger().info("Published IMU data")

    def publish_grippers(self):
        state = random.choice([True, False])
        
        msg_r = Bool()
        msg_r.data = state
        self.gripper_r_pub.publish(msg_r)
        
        msg_l = Bool()
        msg_l.data = not state
        self.gripper_l_pub.publish(msg_l)
        self.get_logger().info(f"Published grippers: R={state}, L={not state}")

    def publish_float(self):
        msg = Float64()
        msg.data = round(random.uniform(0, 50), 2)
        self.float_pub.publish(msg)
        self.get_logger().info(f"Published float depth: {msg.data}m")

def main(args=None):
    rclpy.init(args=args)
    publisher = TestPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("Shutting down test publisher...")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()