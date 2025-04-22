import rclpy
from rclpy.node import Node
import rclpy.node
from sensor_msgs.msg import Imu

from std_msgs.msg import Bool, Float64, Int32MultiArray, Int8, Float32
from geometry_msgs.msg import Vector3, Twist

import threading
from PyQt5.QtCore import QObject, pyqtSignal
import time


class SignalSender(QObject):
    depth_signal = pyqtSignal(float)
    thrusters_signal = pyqtSignal(list)
    gripper_r_signal = pyqtSignal(bool)
    gripper_l_signal = pyqtSignal(bool)
    imu_signal = pyqtSignal(Imu)
    float_signal = pyqtSignal(float)
    angles_signal = pyqtSignal(Vector3)
    desired_signal = pyqtSignal(Twist)



class ROSInterface(Node):
    def __init__(self):
        super().__init__("ros_interface")
        self.signal_emitter = SignalSender()

        # Subscribers
        self.depth_sub = self.create_subscription(
            Float32, "/ROV/depth", self.depth_callback, 10
        )
        self.thrusters_sub = self.create_subscription(
            Int32MultiArray, "ROV/thrusters", self.thrusters_callback, 10
        )
        self.gripper_r_sub = self.create_subscription(
            Bool, "ROV/gripper_r", self.gripper_r_callback, 10
        )
        self.gripper_l_sub = self.create_subscription(
            Bool, "ROV/gripper_l", self.gripper_l_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "ROV/imu", self.imu_callback, 10)

        self.angles_sub = self.create_subscription(
            Vector3, "ROV/angles", self.angles_callback, 10
        )
        self.float_sub = self.create_subscription(
            Float64, "Float/depth", self.float_callback, 10
        )
        self.desired_sub = self.create_subscription(
            Twist, "ROV/desired", self.desired_callback, 10
        )
        # Publishers
        self.pumb_publisher = self.create_publisher(Int8, "/ROV/pump", 10)
        self.test_pub = self.create_publisher(Float64, "/ROV/test", 10)

    # Callbacks
    def depth_callback(self, msg):
        self.signal_emitter.depth_signal.emit(msg.data)

    def thrusters_callback(self, msg):
        self.signal_emitter.thrusters_signal.emit(list(msg.data))

    def gripper_r_callback(self, msg):
        self.signal_emitter.gripper_r_signal.emit(msg.data)

    def gripper_l_callback(self, msg):
        self.signal_emitter.gripper_l_signal.emit(msg.data)

    def imu_callback(self, msg):
        self.signal_emitter.imu_signal.emit(msg)

    def float_callback(self, msg):
        self.signal_emitter.float_signal.emit(msg.data)

    def angles_callback(self, msg):
        self.signal_emitter.angles_signal.emit(msg)
    
    def desired_callback(self, msg):
        self.signal_emitter.desired_signal.emit(msg)
class ROSThread(threading.Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.daemon = True  # Proper daemon thread handling

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()


def start_ros() -> ROSInterface:
    rclpy.init()
    ros_interface = ROSInterface()
    ROSThread(ros_interface).start()
    return ros_interface


# if __name__ == '__main__':
#     # Initialize ROS
#     node = start_ros()
#     ros_thread = ROSThread(node)
#     ros_thread.start()

#     try:
#         # Main thread simply waits while ROS runs in background
#         while ros_thread.is_alive():
#             time.sleep(0.1)
#     except KeyboardInterrupt:
#         print("\nShutting down...")
#     finally:
#         # Cleanup resources
#         ros_thread.stop()
#         if rclpy.ok():
#             rclpy.shutdown()
#         print("ROS node terminated cleanly")
