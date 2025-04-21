#import important libraries and modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from .PySticks.pysticks import get_controller


# Define a ROS2 node class for handling joystick input
class JoyStickNode(Node): 
    def __init__(self):
        super().__init__("joystick")
        #Create a publisher for sending Twist messages on the "ROV/joystick" topic with a queue size of 10
        self.joystick_publisher = self.create_publisher(Twist, "ROV/joystick", 10)

        #Create a publisher for sending Bool messages on the gripper topics with a queue size of 1
        self.gripper_r_publisher = self.create_publisher(Bool, "ROV/gripper_r", 1)
        self.gripper_l_publisher = self.create_publisher(Bool, "ROV/gripper_l", 1)

        #Create a timer to call the update function every 0.01 seconds
        self.timer = self.create_timer(0.01, self.update)

        # Initialize the controller using the get_controller() function from the PySticks module
        self.controller = get_controller()

        # Initialize empty Twist and Gripper messages that will be used for publishing in the update loop
        self.twist_msg = Twist() # For joystick movement commands
        self.gripper_r_msg = Bool()  # For right gripper state
        self.gripper_l_msg = Bool() # For left gripper state

    # The update function is called by the timer every 0.01 seconds
    def update(self):

        # Process new joystick events (using pygame's event pump)
        self.controller.update()
        
        # Read the state of the D-pad (hat switch)
        # getAimball() from PySticks.py returns a tuple (x, y); we unpack them into aim_ball_x and aim_ball_y.
        aim_ball_x = self.controller.getAimball()[0]
        aim_ball_y = self.controller.getAimball()[1]

        # Read the depth controls:
        # depthUp() and depthDown() from PySticks.py update the controller's depth attribute based on button presses.
        self.controller.depthUp()
        self.controller.depthDown()

        # Update the gripper states (toggle left/right gripper using button presses) 
        # toggle action is managed in PySticks.py
        self.controller.leftGripper()
        self.controller.rightGripper()

        # Set the linear.x component of the twist message (forward/backward movement)
        # Only use the joystick's pitch value if the D-pad (aim_ball_x) is not active; else, set to 0.
        self.twist_msg.linear.x = (
            float(self.controller.getPitch()) if not aim_ball_x else 0.0
        )

        # Set the linear.y component of the twist message (left/right movement)
        # Only use the joystick's roll value if the D-pad (aim_ball_y) is not active; else, set to 0.
        self.twist_msg.linear.y = (
            float(self.controller.getRoll()) if not aim_ball_y else 0.0
        )

        # Set the linear.z component (vertical movement or depth)
        # This is directly taken from the controller's depth value affected by calling depthUp() and depthDown()
        self.twist_msg.linear.z = float(self.controller.depth)

        # Set the angular.z component (yaw)
        # Only use the joystick's yaw value if the D-pad (aim_ball_x and aim_ball_y) is not active; else, set to 0.
        self.twist_msg.angular.z = (
            float(self.controller.getYaw())
            if not aim_ball_x and not aim_ball_y
            else 0.0
        )

        # Update gripper message states:
        # These Boolean values represent the current state (open/closed) of the right and left grippers.
        self.gripper_r_msg.data = self.controller.right_gripper
        self.gripper_l_msg.data = self.controller.left_gripper

        # self.get_logger().info("Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   Yaw: %+2.2f   Aux: %+2.2f"
        #     % (
        #         self.controller.depth,
        #         self.controller.getRoll(),
        #         self.controller.getPitch(),
        #         self.controller.getYaw(),
        #         bool(self.controller.getTrigger() + 1),
        #     ))

        # Publish the gripper states to their respective topics
        self.gripper_r_publisher.publish(self.gripper_r_msg)
        self.gripper_l_publisher.publish(self.gripper_l_msg)

        # Check if the stop button is pressed; if so, reset the twist message to a new, empty Twist (all zeros)
        if self.controller.stopAll():
            self.twist_msg = Twist()

        # Publish the movement command (Twist message) to the "ROV/joystick" topic
        self.joystick_publisher.publish(self.twist_msg)

# The main function to initialize and run the JoyStickNode
def main(args=None):
    rclpy.init(args=args)          # Initialize the ROS2 Python client library

    jotstick_node = JoyStickNode() # Create an instance of the JoyStickNode
    rclpy.spin(jotstick_node)      # Keep the node running to listen and process incoming messages

    jotstick_node.destroy_node()   # Clean up the node when shutting down
    rclpy.shutdown()               # Shut down the ROS2 client library


if __name__ == "__main__":
    main()
