# ros_logic.py
import rclpy
from rclpy.node import Node
from rov_debug_interfaces.msg import DecodedData, Imu
from PyQt5.QtCore import pyqtSignal, QObject
from std_msgs.msg import Float32
import threading

class SignalSender(QObject):
    # Signal will emit the complete DecodedData message.
    custom_signal = pyqtSignal(DecodedData)
    depth_signal = pyqtSignal(float)

class ROSInterface(Node):
    def __init__(self):
        super().__init__('ros_interface')
        self.subscription = self.create_subscription(
            DecodedData,
            'data',
            self.listener_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Float32,
            '/rov/depth',
            self.depth_callback,
            10)
        
        self.subscription  # To avoid unused variable warning.
        self.signal_emitter = SignalSender()

    def listener_callback(self, msg):
        # Emit a signal carrying the complete message.
        self.signal_emitter.custom_signal.emit(msg)
    def depth_callback(self, msg):
        self.signal_emitter.depth_signal.emit(msg.data)

def run_ros_node(ros_interface):
    rclpy.spin(ros_interface)

def start_ros():
    rclpy.init()
    ros_interface = ROSInterface()
    # Run ROS spin in a separate thread so it doesn't block the GUI.
    ros_thread = threading.Thread(target=run_ros_node, args=(ros_interface,), daemon=True)
    ros_thread.start()
    return ros_interface
if __name__ == '__main__':
    node = start_ros()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    except Exception as e:
        print(f"Unexpected exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()



