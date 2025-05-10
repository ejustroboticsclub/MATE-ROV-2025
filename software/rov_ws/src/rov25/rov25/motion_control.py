# Import necessary libraries and modules:
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



# Define a dataclass to hold tuning and utility parameters for motion control.

@dataclass
class Param:
    """tuning and utils params"""

    vx_const: float = 1         # Scaling factor for forward/backward velocity (linear.x)
    vy_const: float = 1         # Scaling factor for left/right velocity (linear.y)
    wz_const: float = 1         # Scaling factor for angular velocity (angular.z)
    dz_const: float = 1         # Scaling factor for depth velocity (linear.z)
    pool_depth: float = 1.0     # Maximum pool depth; value can be changed (e.g., 4 for competition)

# Create an instance of Param with default values.
PARAM = Param()


# ------------------------------------------------------------------------------
# Define the MotionControl node class, which processes joystick commands and
# publishes final velocity commands to the ROV.
# ------------------------------------------------------------------------------
class MotionControl(Node):
    def __init__(self):
        # Initialize the MotionControl node with the name "motion_control".
        super().__init__("motion_control")

        # Create a publisher for sending Twist messages on the "ROV/cmd_vel" topic with a queue size of 10.
        # The node movement_feedback.py subscribes to this topic to recieve final velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, "ROV/cmd_vel", 10)

        # Create a subscription to the "ROV/joystick" topic.
        # This topic receives raw joystick commands (as Twist messages) from the joystick node.
        # When a new message is received, the callback 'ctrl_sig_recieved_callback' is triggered.
        self.create_subscription(
            Twist, "ROV/joystick", self.ctrl_sig_recieved_callback, 10
        )

     
    # Callback function for processing incoming Twist messages from "ROV/joystick"
    def ctrl_sig_recieved_callback(self, twist_msg: Twist):
        """callback function for the subscriber to the ROV/joystick topic

        Args:
            msg (Twist): Twist message sent by the joystick
        """

        # Extract the individual components from the incoming Twist message.
        v_x = twist_msg.linear.x        # Raw forward/backward input
        v_y = twist_msg.linear.y        # Raw left/right input
        d_z = twist_msg.linear.z        # Raw depth input
        w_z = twist_msg.angular.z       # Raw Yaw input

        # Scale the raw inputs using the tuning parameters
        v_x = PARAM.vx_const * v_x      # desired v_x
        v_y = PARAM.vy_const * v_y      # desired v_y

        # For depth (vertical movement):
        # First, scale by the pool depth to account for different pool sizes.
        d_z = PARAM.pool_depth * d_z    # desired d_z

        # d_z = map_from_to(d_z, -1, 1, 1, -1)  # desired d_z

        # Then, apply an additional depth scaling factor.
        d_z = PARAM.dz_const * d_z      # desired d_z

        # Scale the yaw rotation input.
        w_z = PARAM.wz_const * w_z      # desired w_z

        # Create a new Twist message to send as the final, processed command.
        vel = Twist()

        # Set the linear and angular components of the Twist message.
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = w_z             # desired w_z
        vel.linear.x = v_x              # desired v_x
        vel.linear.y = v_y              # desired v_y
        vel.linear.z = d_z              # desired d_z

        # Publish the final Twist message to the "ROV/cmd_vel" topic, where movement_feedback.py is subscribed.
        self.cmd_vel_publisher.publish(vel)

# Main function to initialize and run the MotionControl node.
def main(args=None):
    rclpy.init(args=args)                # Initialize the ROS2 Python client library.

    motion_control_node = MotionControl()# Create an instance of the MotionControl node.
    rclpy.spin(motion_control_node)      # Keep the node running to listen and process incoming messages.

    rclpy.shutdown()


if __name__ == "__main__":
    main()
