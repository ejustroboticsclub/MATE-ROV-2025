import rclpy
from rclpy.node import Node
#TODO: switch the imports when the node is deployed
# from calibartion_node import live_queue_plotter as que
import live_queue_plotter as que
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import Imu
import keyboard
import numpy as np 


max_kp = 7
min_kp = 0.01
current = 0
numerical_input = False
single = False
# the constants are open vx, open vy, kp_wz, kp_depth, kp_roll, kp_pitch
constants_list = []
updated = 0 
choosing = False


def keyboard_loop():
    global numerical_input, single, constant, constants_list, updated, choosing
    while True:
        if not numerical_input:
            if keyboard.is_pressed("esc"):
                break
            elif keyboard.is_pressed("a"):
                numerical_input = True
                single = False
            elif keyboard.is_pressed("q"):
                numerical_input = True
                single = True
            elif keyboard.is_pressed("c"):
                numerical_input = True
                choosing = True
            elif keyboard.is_pressed("w"):
                constant+=0.05
                if constant>max_kp:
                    constant = max_kp
                updated = 1
            elif keyboard.is_pressed("s"):
                constant -= 0.05
                if constant<min_kp:
                    constant = min_kp
                updated = 1
            elif keyboard.is_pressed("up"):
                constant += 0.01
                if constant>max_kp:
                    constant = max_kp
                updated = 1
            elif keyboard.is_pressed("down"):
                constant -= 0.01
                if constant<min_kp:
                    constant = min_kp
                updated = 1
            
            
        else:
            if not choosing:
                if single:
                    constant = int(input(f"Enter a value for constant {current}: "))
                    updated = 1
                else:
                    constants_list = list(map(int, input("enter all the constants seperated with spaces: \n").split()))
                    updated = 2
            else:
                current = int(input("Choose constant to control from 0 to 5 \n order: open vx, open vy, kp_wz, kp_depth, kp_roll, kp_pitch \n"))
                updated = 3
                choosing = False
            numerical_input = False        


class Callibration(Node):
    def __init__(self):
        super().__init__("Callibration")
        self.depth_subscribtion = self.create_subscription(Float64, "ROV/depth", self.depth_received_callback, 10)
        self.imu_subscribtion = self.create_subscription(Imu, "ROV/imu", self.imu_received_callback, 10)
        self.depth_subscribtion
        self.imu_subscribtion
        self.constants_publisher = self.create_publisher(Float32MultiArray, "ROV/constants", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.constants = constants_list
        self.spawn_plots() 
    
    #TODO: correctly set the queue size and duration
    def spawn_plots():
        que.initialize_queue(100, 10)

    def timer_callback(self):
        global updated
        if updated == 0:
            return
        else:
            if updated == 1:
                self.constants[current] = constant
            elif updated == 2:
                self.constants = constants_list
            elif updated == 3:
                self.spawn_plots()
            message = Float32MultiArray()
            message.data = self.constants
            self.constants_publisher.publish(message)
            updated = 0
            
    def depth_received_callback(self, mesg: Float64):
        if current == 3:
            que.update(mesg.data)
    
    def imu_received_callback(self, mesg: Imu):
        if current != 3:
            if current==2:
                que.update(mesg.angular_velocity.z)
            elif current==4:
                que.update(mesg.angular_velocity.x)
            elif current == 5:
                que.update(mesg.angular_velocity.y)

