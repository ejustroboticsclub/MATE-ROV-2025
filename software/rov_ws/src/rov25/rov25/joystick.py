#!/usr/bin/env python3
#import important libraries and modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
import pygame
import time


"""
pysticks.py: Python classes for flying with joysticks, R/C controllers

Requires: pygame

Copyright (C) Simon D. Levy 2016

This file is part of PySticks.

PySticks is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this PySticks.  If not, see <http:#www.gnu.org/licenses/>.
"""


class Controller(object):
    STICK_DEADBAND = 0.05

    def __init__(self, axis_map):
        self.joystick = None
        self.axis_map = axis_map

    def update(self):
        pygame.event.pump()


    def getThrottle(self):
        return self._getAxis(0)

    def getRoll(self):
        return self._getAxis(1)

    def getPitch(self):
        return self._getAxis(2)

    def getYaw(self):
        return self._getAxis(3)

    def _getAxis(self, k):
        j = self.axis_map[k]
        val = self.joystick.get_axis(abs(j))
        if abs(val) < Controller.STICK_DEADBAND:
            val = 0
        return (-1 if j < 0 else +1) * val

class _GameController(Controller):
    def __init__(self, axis_map, button_id):
        Controller.__init__(self, axis_map)
        
        self.button_id = button_id
        self.button_is_down = False
        
        self.switch_value = -1
        
        self.depth = 0
        
        self.right_gripper = False
        self.right_gripper_button_is_down = False
        
        self.left_gripper = False
        self.left_gripper_button_is_down = False
        
        self.rotating_button=False
        self.rotating_button_is_down=False
        
        self.pitch_button = False
        self.pitch_button_is_down = False

    def _getAuxValue(self):
        return self.joystick.get_button(self.button_id)

    def getTrigger(self):
        if self._getAuxValue():
            if not self.button_is_down:
                self.switch_value = -self.switch_value
            self.button_is_down = True
        else:
            self.button_is_down = False
        return self.switch_value

    def getAimball(self):
        """Reads the value of the small ball (hat) on the top

        Returns:
            x and y state (tuple): the active directions of x and y (-1 or 1) 
        """
        return self.joystick.get_hat(0)
    
    def depthDown(self):
        # Don't rise up if the maximum value reached
        if self.joystick.get_button(4):
            self.depth += 0.005
        self.depth = min(1, self.depth)

    
    def depthUp(self):
        # Don't rise up if the maximum value reached
        if self.joystick.get_button(5):
            self.depth -= 0.005
        self.depth = max(0, self.depth)

    def leftGripper(self):
        if self.joystick.get_button(2):
            if not self.left_gripper_button_is_down:
                self.left_gripper = not self.left_gripper
            self.left_gripper_button_is_down = True
        else:
            self.left_gripper_button_is_down = False

    
    def rightGripper(self):
        if self.joystick.get_button(3):
            if not self.right_gripper_button_is_down:
                self.right_gripper = not self.right_gripper
            self.right_gripper_button_is_down = True
        else:
            self.right_gripper_button_is_down = False

    def stopAll(self):
        # Breaks button
        return bool(self.joystick.get_button(1))
        ###
        
    def rotatingROV(self):
        """button for rotating the ROV 360 degree around itself"""
        if self.joystick.get_button(9):  
            if not self.rotating_button_is_down:
                self.rotating_button=not self.rotating_button
            self.rotating_button_is_down = True
        else:
            self.rotating_button_is_down = False
    

    def pitchCheck(self):
        """button for enabling the pitch publisher"""
        if self.joystick.get_button(8):
            print("button 10 pressed")
            if not self.pitch_button_is_down:
                self.pitch_button = not self.pitch_button
            self.pitch_button_is_down = True
        else:
            self.pitch_button_is_down = False

    
        
class _SpringyThrottleController(_GameController):
    def __init__(self, axis_map, button_id):
        _GameController.__init__(self, axis_map, button_id)

        self.throttleval = -1

        self.prevtime = 0

    def getThrottle(self):
        currtime = time.time()

        # Scale throttle increment by time difference from last update
        self.throttleval += self._getAxis(0) * (currtime - self.prevtime)

        # Constrain throttle to [-1,+1]
        self.throttleval = min(max(self.throttleval, -1), +1)

        self.prevtime = currtime

        return self.throttleval


class _RcTransmitter(Controller):
    def __init__(self, axis_map, aux_id):
        Controller.__init__(self, axis_map)
        self.aux_id = aux_id

    def getAux(self):
        return +1 if self.joystick.get_axis(self.aux_id) > 0 else -1


class _Xbox360(_SpringyThrottleController):
    def __init__(self, axes, aux):
        _SpringyThrottleController.__init__(self, axes, None)

        self.aux = aux

    def _getAuxValue(self):
        return self.joystick.get_axis(self.aux) < -0.5


class _Playstation(_SpringyThrottleController):
    def __init__(self, axes):
        _SpringyThrottleController.__init__(self, axes, 7)


# Different OSs have different names for the same controller, so we don't
# need to check OS when setting up the axes.
controllers = {
    "Controller (Rock Candy Gamepad for Xbox 360)": _Xbox360((-1, 4, -3, 0), 2),
    "Rock Candy Gamepad for Xbox 360": _Xbox360((-1, 3, -4, 0), 5),
    "2In1 USB Joystick": _Playstation((-1, 2, -3, 0)),
    "Wireless Controller": _Playstation((-1, 2, -3, 0)),
    "MY-POWER CO.,LTD. 2In1 USB Joystick": _Playstation((-1, 2, -3, 0)),
    "Sony Interactive Entertainment Wireless Controller": _Playstation((-1, 3, -4, 0)),
    "Logitech Extreme 3D": _GameController((-2, 0, -1, 3), 0),
    "Logitech Logitech Extreme 3D": _GameController((-3, 0, -1, 2), 0),
    "Logitech Extreme 3D pro": _GameController((-3, 0, -1, 2), 0),
    "FrSky Taranis Joystick": _RcTransmitter((0, 1, 2, 5), 3),
    "FrSky FrSky Taranis Joystick": _RcTransmitter((0, 1, 2, 3), 5),
    "SPEKTRUM RECEIVER": _RcTransmitter((1, 2, 5, 0), 4),
    "Horizon Hobby SPEKTRUM RECEIVER": _RcTransmitter((1, 2, 3, 0), 4),
}


def get_controller():
    # Initialize pygame for joystick support
    pygame.display.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Find your controller
    controller_name = joystick.get_name()
    if not controller_name in controllers.keys():
        print("Unrecognized controller: %s" % controller_name)
        exit(1)
    controller = controllers[controller_name]
    controller.joystick = joystick

    return controller


# Define a ROS2 node class for handling joystick input
class JoyStickNode(Node): 
    def __init__(self):
        super().__init__("joystick")
        #Create a publisher for sending Twist messages on the "ROV/joystick" topic with a queue size of 10
        self.joystick_publisher = self.create_publisher(Twist, "ROV/joystick", 10)

        #Create a publisher for sending Bool messages on the gripper topics with a queue size of 1
        self.gripper_r_publisher = self.create_publisher(Bool, "ROV/gripper_r", 1)
        self.gripper_l_publisher = self.create_publisher(Bool, "ROV/gripper_l", 1)

        # Publisher for the rotating Mode.
        self.rotatingROV_publisher=self.create_publisher(Bool, "ROV/rotating", 1)

        # Publisher to set Pitch
        self.pitch_publisher = self.create_publisher(
            Int32, "ROV/pitch", 1
        )
        # Subscriber to know if the ROV reached 360 rotation so it can set rotating_button to False.
        self.rotation_done = self.create_subscription(
            Bool, "ROV/rotation_done", self.rotation_done_callback, 10
        )
       
        #Create a timer to call the update function every 0.01 seconds
        self.timer = self.create_timer(0.01, self.update)

        # Initialize the controller using the get_controller() function from the PySticks module
        self.controller = get_controller()

        # Initialize empty Twist and Gripper messages that will be used for publishing in the update loop
        self.twist_msg = Twist() # For joystick movement commands
        
        self.gripper_r_msg = Bool()  # For right gripper state
        self.gripper_r_msg.data = False
        self.prev_gripper_r = False

        self.gripper_l_msg = Bool() # For left gripper state
        self.gripper_l_msg.data = False 
        self.prev_gripper_l = False


        # Message and flags for rotation mode.
        self.rotatingROV_msg = Bool()
        self.rotatingROV_msg.data = False
        self.prev_rotating = False

        # Messages and flags for pitch mode.
        self.pitch_msg = Int32()
        self.pitch_msg.data = 0
        self.prev_pitch = 0




    def rotation_done_callback(self, msg):
        """Callback function to handle the rotation done message"""
        # If the rotation is done, set the rotating button to False
        if msg.data:
            self.controller.rotating_button = False
            self.rotatingROV_msg.data = False
            self.prev_rotating = False    
    
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


        # Update the rotation mode status. (roating \ not rotating)
        self.controller.rotatingROV()
        self.controller.pitchCheck()

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

        #Update the message to match the status from the button
        self.rotatingROV_msg.data = self.controller.rotating_button

        # self.get_logger().info("Throttle: %+2.2f   Roll: %+2.2f   Pitch: %+2.2f   Yaw: %+2.2f   Aux: %+2.2f"
        #     % (
        #         self.controller.depth,
        #         self.controller.getRoll(),
        #         self.controller.getPitch(),
        #         self.controller.getYaw(),
        #         bool(self.controller.getTrigger() + 1),
        #     ))

        # Publish the gripper states to their respective topics
        if self.gripper_r_msg.data != self.prev_gripper_r:
            self.gripper_r_publisher.publish(self.gripper_r_msg)
            self.prev_gripper_r = self.gripper_r_msg.data
        
        if self.gripper_l_msg.data != self.prev_gripper_l:
            self.gripper_l_publisher.publish(self.gripper_l_msg)
            self.prev_gripper_l = self.gripper_l_msg.data

        # Publish the rotating mode message.
        if self.rotatingROV_msg.data != self.prev_rotating: 
            self.rotatingROV_publisher.publish(self.rotatingROV_msg)
            self.prev_rotating = self.rotatingROV_msg.data 


        if self.controller.pitch_button:
            self.pitch_msg.data = 45
        else:
            self.pitch_msg.data = 0
         
        # Publish the pitch message only  if the pitch value has changed.        
        if self.pitch_msg.data != self.prev_pitch:
            self.pitch_publisher.publish(self.pitch_msg)
            self.prev_pitch = self.pitch_msg.data
        else:
            self.pitch_msg.data = 0

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
