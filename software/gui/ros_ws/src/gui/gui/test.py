import rclpy
from rclpy.node import Node
import random
from rov_debug_interfaces.msg import DecodedData, Imu
from std_msgs.msg import Float32  # Add Float32 for depth

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        # Existing DecodedData publisher
        self.publisher_ = self.create_publisher(DecodedData, 'data', 10)
        # New depth publisher
        self.depth_publisher = self.create_publisher(Float32, '/rov/depth', 10)
        
        # Timer for regular DecodedData messages
        self.timer = self.create_timer(1.0, self.publish_message)
        # Timer for random depth messages (initial setup)
        self.depth_timer = None
        self.trigger_depth_publish()
        
        self.counter = 0

    def trigger_depth_publish(self):
        """Schedule next depth publication with random delay"""
        if self.depth_timer:
            self.depth_timer.cancel()
        interval = random.uniform(0.5, 3.0)  # Random interval between 0.5-3 seconds
        self.depth_timer = self.create_timer(interval, self.publish_depth)

    def publish_depth(self):
        """Publish random depth value and schedule next"""
        depth_msg = Float32()
        depth_msg.data = round(random.uniform(0, 100), 2)
        self.depth_publisher.publish(depth_msg)
        self.get_logger().info(f"Published depth: {depth_msg.data}m")
        self.trigger_depth_publish()  # Reschedule with new random delay

    def publish_message(self):
        """Existing DecodedData publisher (unchanged)"""
        msg = DecodedData()
        msg.id = self.counter
        msg.valid = random.choice([True, False])
        
        # Populate the nested Imu message within DecodedData.
        imu_msg = Imu()
        imu_msg.acc_x = round(random.uniform(-1.0, 1.0), 2)
        imu_msg.acc_y = round(random.uniform(-1.0, 1.0), 2)
        imu_msg.acc_z = round(random.uniform(8.0, 10.0), 2)
        imu_msg.imu_roll = round(random.uniform(-180.0, 180.0), 2)
        imu_msg.imu_pitch = round(random.uniform(-90.0, 90.0), 2)
        imu_msg.imu_yaw = round(random.uniform(-180.0, 180.0), 2)
        msg.imu = imu_msg
        
        # Thruster currents
        for i in range(1, 7):
            setattr(msg, f"thruster_current_{i}", round(random.uniform(0, 50), 2))
        # Thruster PWM
        for i in range(1, 7):
            setattr(msg, f"thruster_pwm_{i}", round(random.uniform(1000, 2000), 2))
        # Indicators
        for i in range(1, 7):
            setattr(msg, f"indicator_{i}", round(random.uniform(0, 100), 2))
        # Heartbeats
        for i in range(1, 5):
            setattr(msg, f"heartbeat_{i}", random.choice([True, False]))
        # Connection percentages
        for i in range(1, 5):
            setattr(msg, f"connection_percentage_{i}", round(random.uniform(0, 100), 2))
        # Arms
        for i in range(1, 5):
            setattr(msg, f"arm_{i}", round(random.uniform(0, 1), 2))
        # rov_depth (keep this if needed for other purposes)
        msg.rov_depth = round(random.uniform(0, 100), 2)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published DecodedData msg id: {msg.id}")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()