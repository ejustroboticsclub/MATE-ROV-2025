import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from .PySticks.pysticks import get_controller


class JoyStickNode(Node):
    def __init__(self):
        super().__init__("joystick")

        self.joystick_publisher = self.create_publisher(Twist, "ROV/joystick", 10)
        self.gripper_r_publisher = self.create_publisher(Bool, "ROV/gripper_r", 1)
        self.gripper_l_publisher = self.create_publisher(Bool, "ROV/gripper_l", 1)
        self.timer = self.create_timer(0.01, self.update)
        self.controller = get_controller()

        self.twist_msg = Twist()
        self.gripper_r_msg = Bool()
        self.gripper_l_msg = Bool()

    def update(self):
        self.controller.update()

        aim_ball_x = self.controller.getAimball()[0]
        aim_ball_y = self.controller.getAimball()[1]

        # Get depth value readings
        self.controller.depthUp()
        self.controller.depthDown()

        self.controller.leftGripper()
        self.controller.rightGripper()

        self.twist_msg.linear.x = (
            float(self.controller.getPitch()) if not aim_ball_x else 0.0
        )
        self.twist_msg.linear.y = (
            float(self.controller.getRoll()) if not aim_ball_y else 0.0
        )
        self.twist_msg.linear.z = float(self.controller.depth)
        self.twist_msg.angular.z = (
            float(self.controller.getYaw())
            if not aim_ball_x and not aim_ball_y
            else 0.0
        )

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

        self.gripper_r_publisher.publish(self.gripper_r_msg)
        self.gripper_l_publisher.publish(self.gripper_l_msg)

        if self.controller.stopAll():
            self.twist_msg = Twist()

        self.joystick_publisher.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)

    jotstick_node = JoyStickNode()
    rclpy.spin(jotstick_node)

    jotstick_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
