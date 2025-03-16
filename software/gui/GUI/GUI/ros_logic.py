import rclpy
from rclpy.node import Node
from rov_debug_interfaces.msg import DecodedData

class ROSInterface(Node):
    def __init__(self):
        super().__init__('ros_interface')
        self.subscription = self.create_subscription(
            DecodedData,
            'data',   # Topic name
            self.listener_callback,
            10
        )
        self.subscription
        self.latest_msg = None

    def listener_callback(self, msg: DecodedData):
        self.get_logger().info(f"Received DecodedData msg id: {msg.id}")
        self.latest_msg = msg

def main_ros(args=None):
    rclpy.init(args=args)
    ros_interface = ROSInterface()
    return ros_interface

if __name__ == '__main__':
    node = ROSInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
