import dataclasses
from time import time
import math
from typing import List
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy.parameter
from std_msgs.msg import Float64, Float32MultiArray,Bool
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import SetParametersResult

PARAMETERS = [
    "x.kp",
    "x.ki",
    "x.kd",
    "y.kp",
    "y.ki",
    "y.kd",
    "w.kp",
    "w.ki",
    "w.kd",
    "depth.kp",
    "depth.ki",
    "depth.kd",
    "roll.kp",
    "roll.ki",
    "roll.kd",
    "floatability",
    "pitch.kd",
    "pitch.ki",
    "pitch.kp",
    ###
    "yaw.kp",
    "yaw.ki",
    "yaw.kd"
    ###
]


@dataclasses.dataclass
class ErrorVal:
    """PID error terms"""

    e_sum: float = 0
    d_error: float = 0
    current_error: float = 0
    prev_error: float = 0


class PID:
    """PID controller Class"""

    def __init__(
        self,
        k_proportaiol: float,
        k_integral: float,
        k_derivative: float,
        windup_val: float,
    ) -> None:
        """Creates a PID controller using provided PID parameters

        Args:
            k_proportional (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative
        self.pid: float = 0
        self.error = ErrorVal()
        self.windup_val = windup_val

    def compute(self, ref: float, measured: float) -> float:
        """Computes the PID value based on the given reference and measured output values

        Args:
            ref (float): reference signal that we desire to track
            measured (float): actual measured output of the signal

        Returns:
            float: PID value
        """
        self.error.current_error = ref - measured
        self.error.e_sum += self.error.current_error
        self.error.d_error = self.error.current_error - self.error.prev_error
        self.pid = (
            self.k_proportaiol * self.error.current_error
            + self.k_integral * self.error.e_sum
            + self.k_derivative * self.error.d_error
        )
        self.error.prev_error = self.error.current_error
        if self.error.current_error <= self.windup_val:
            self.error.e_sum = 0
        return self.pid

    def set_pid(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Sets the PID controller constants

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative


def map_from_to(
    num: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """Mapping function

    Args:
        num (float): Value to be mapped
        in_min (float): Minimum value of input
        in_max (float): Maximum value of input
        out_min (float): Minimum value of ouput
        out_max (float): Maximum value of ouput

    Returns:
        float: Mapped value
    """
    return (
        float(num - in_min) / float(in_max - in_min) * (out_max - out_min)
    ) + out_min


@dataclasses.dataclass
class Param:
    """Tuning and utils params"""

    kp_x: float
    ki_x: float
    kd_x: float
    kp_y: float
    ki_y: float
    kd_y: float
    kp_w: float
    ki_w: float
    kd_w: float
    kp_depth: float
    ki_depth: float
    kd_depth: float
    kp_roll: float
    ki_roll: float
    kd_roll: float
    ###
    kp_yaw: float
    ki_yaw: float
    kd_yaw: float
    ###
    kp_pitch: float
    kd_pitch: float
    ki_pitch: float
    floatability: float

    windup_val_x = 0.1
    windup_val_y = 0.1
    windup_val_w = 0.1
    windup_val_depth = 0.1
    windup_val_roll = 0.1
    windup_val_pitch = 0.1
    windup_val_yaw = 0.1

    thruster_max = 1660
    thruster_min = 1310  # Changed to 1180 instead of 1160 to make sure that the center is 1485 which is a stoping value

    thruster_side_max = 1700
    thruster_side_min = 1300

    # corredponding to motion control node
    # TODO: adjust the PARAM.min and PARAM.max to all 3 motions
    max_vx = 3
    min_vx = -3
    max_vy = 3
    min_vy = -3
    max_wz = 0.7
    min_wz = -0.7
    max_pool_depth = 1
    max_roll = 90
    min_roll = -90
    ###
    min_yaw=-180
    max_yaw=180
    ###

    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    angular_offset = 0


@dataclasses.dataclass
class Measured:
    """Sensor measurments"""

    v_x = 0
    v_y = 0
    w_z = 0
    yaw = 0
    roll = 0
    depth = 0
    pitch = 0


@dataclasses.dataclass
class Reference:
    """desired values"""

    v_x = 0
    v_y = 0
    w_z = 0
    roll = 0
    depth = 0
    pitch = 0
    ###
    yaw=0
    ###



@dataclasses.dataclass
class Time:
    """Time related variables for integration and differentiation"""

    t_prev = 0
    t_current = 0
    delta_t = 0


def create_pid_objects(parameter_object: Param):
    pid_vx = PID(
        k_proportaiol=parameter_object.kp_x,
        k_derivative=parameter_object.kd_x,
        k_integral=parameter_object.ki_x,
        windup_val=parameter_object.windup_val_x,
    )
    pid_vy = PID(
        k_proportaiol=parameter_object.kp_y,
        k_derivative=parameter_object.kd_y,
        k_integral=parameter_object.ki_y,
        windup_val=parameter_object.windup_val_y,
    )
    pid_wz = PID(
        k_proportaiol=parameter_object.kp_w,
        k_derivative=parameter_object.kd_w,
        k_integral=parameter_object.ki_w,
        windup_val=parameter_object.windup_val_w,
    )
    pid_depth = PID(
        k_proportaiol=parameter_object.kp_depth,
        k_derivative=parameter_object.kd_depth,
        k_integral=parameter_object.ki_depth,
        windup_val=parameter_object.windup_val_depth,
    )
    pid_stabilization = PID(
        k_proportaiol=parameter_object.kp_roll,
        k_derivative=parameter_object.kd_roll,
        k_integral=parameter_object.ki_roll,
        windup_val=parameter_object.windup_val_roll,
    )
    pid_pitch = PID(
        k_proportaiol= parameter_object.kp_depth,
        k_derivative=parameter_object.kd_depth,
        k_integral=parameter_object.ki_depth,
        windup_val=parameter_object.windup_val_pitch
    )
    ###
    pid_yaw= PID(  
        k_proportaiol=parameter_object.kp_yaw,
        k_derivative=parameter_object.kd_yaw,
        k_integral=parameter_object.ki_yaw,
        windup_val=parameter_object.windup_val_yaw,  
    )
    ###

    return pid_vx, pid_vy, pid_wz, pid_depth, pid_stabilization, pid_pitch,pid_yaw


class Robot:
    """Robot Class to solve the kinematic model"""

    @staticmethod
    def kinematic_control(pid_val_x: float, pid_val_y: float, pid_val_w: float ) -> List:
        """
        Calculates the thrusters' values based on their position and the desired
        planner movement

        Args:
            pid_val_x (float): PID value calculated for the error in x velocity
            pid_val_y (float): PID value calculated for the error in y velocity
            pid_val_w (float): PID value calculated for the error in w velocity

        Returns:
            List: List of four thrusters values responsible for the planner kinematic
            motion of the ROV
        """
        # Planer Control
        phi_2 = pid_val_x - pid_val_y
        phi_1 = pid_val_x + pid_val_y
        phi_3 = pid_val_x - pid_val_y
        phi_4 = pid_val_x + pid_val_y

        # Rotation Control
        phi_1 += pid_val_w
        phi_2 += -pid_val_w
        phi_3 += pid_val_w
        phi_4 += -pid_val_w

        return [phi_1, phi_2, phi_3, phi_4]

    @staticmethod
    def find_phi_boudary_values(param: Param) -> List:
        """Estimate the thrusters boundary value solely based on max allowed velocity
            in each axis

        Args:
            param (PARAM): PARAM data class

        Returns:
            List: Boundary values for the thrusters actuation
        """
        max_phi_val = param.max_vx + param.max_vy + param.max_wz
        min_phi_val = param.min_vx + param.min_vy + param.min_wz
        return [min_phi_val, max_phi_val]

    def saturate_actuators(
        self,
        phi_1: float,
        phi_2: float,
        phi_3: float,
        phi_4: float,
        min_phi_val: float,
        max_phi_val: float,
    ) -> List:
        """
        Bounds the value of the thrusters between the max and min values for each one

        Args:
            phi_1 (float): Unsaturated thruster 1 value in the ROV configuration
            phi_2 (float): Unsaturated thruster 2 value in the ROV configuration
            phi_3 (float): Unsaturated thruster 3 value in the ROV configuration
            phi_4 (float): Unsaturated thruster 4 value in the ROV configuration
            min_phi_val (float): Reverse saturation value
            max_phi_val (float): Forward saturation value

        Returns:
            List: Saturated list of four thrusters values responsible for the planner kinematic
            motion of the ROV
        """

        phi_1 = max(phi_1, min_phi_val)
        phi_1 = min(phi_1, max_phi_val)
        phi_2 = max(phi_2, min_phi_val)
        phi_2 = min(phi_2, max_phi_val)
        phi_3 = max(phi_3, min_phi_val)
        phi_3 = min(phi_3, max_phi_val)
        phi_4 = max(phi_4, min_phi_val)
        phi_4 = min(phi_4, max_phi_val)
        return [phi_1, phi_2, phi_3, phi_4]


class CalibrationNode(Node):
    """
        Node That subscribes to the sensors and cmd_vel topics, calculates the thrusters' values, and 
        publishes them to thrusters topic.

        Node name: kinematic_model
    """
    def __init__(self):
        """
            t {Time object}: stores time data for integrating and differentiating

            actual {Measured object}: The actual position and orientation of the rov

            desired {Reference object}: The target position and orientation
        """
        self.t = Time()
        self.actual = Measured()
        self.desired = Reference()
        self.t.t_prev = time()

        ###
        self.is_rotating = False  # Is the ROV currently rotating?
        self.rotation_stage = 0  # 0: Full rotation, 1: Tilt & rotate
        ###

        super().__init__("kinematic_model")
        """
            publisher to ROV/thrusters topic

            subscriber to ROV/cmd_vel with cmd_vel_recieved_callback callback function

            subscriber to ROV/depth with depth_recieved_callback callback 
            
            subscriber to ROV/imu with imu_recieved_callback callback function

            subscriber to ROV/pitch with pitch_received_callback callback function            
        """
        self.thrusters_voltages_publisher = self.create_publisher(
            Float32MultiArray, "ROV/thrusters", 10
        )
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "ROV/cmd_vel", self.cmd_vel_recieved_callback, 10
        )
        self.depth_subscriber = self.create_subscription(
            Float64, "ROV/depth", self.depth_recieved_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, "ROV/imu", self.imu_recieved_callback, 10
        )
        self.pitch_subscriber = self.create_subscription(
            Float64, "ROV/pitch", self.pitch_received_callback, 10
        )
        ###
        self.rotating_button_subscriber = self.create_subscription(
            Bool, "ROV/rotating", self.rotating_button_callback, 10
        )

        ###
        """
            self.timer {Timer object}: timer with timer_callback as callback function that is called 
            every 0.01 sec

            Rov {Robot object}: Calculates the values for the planner thrusters

            self.declare_parameters, self.add_on_set_parameters_callback and parameters_callback 
            set the parameters the declare_parameters takes a list of tuples each having the parameter 
            name and optionally its value. The add_on_set_parameters_callback function is then called for 
            once for each parameter which runs the parameters_callback that uses setattr and updates the 
            parameters dictionary. The log_parameters function is called each time parameters_callback is 
            called.

        """
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.ROV = Robot()

        self.declare_parameters(
            namespace="",
            parameters=[(i, rclpy.Parameter.Type.DOUBLE) for i in PARAMETERS],
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.parameters_dict = {
            i: self.get_parameter(i).get_parameter_value().double_value
            for i in PARAMETERS
        }

        self.PARAM = Param(*self.parameters_dict.values())

        (
            self.pid_vx,
            self.pid_vy,
            self.pid_wz,
            self.pid_depth,
            self.pid_stabilization,
            self.pid_pitch,
            ###
            self.pid_yaw
            ###
        ) = create_pid_objects(self.PARAM)

    def log_parameters(self, parameters_dict: dict):
        """loops over the parameters_dict and logs them using get_logger

        Args:
            parameters_dict (dict): the parameters' dictionary 
        """
        self.get_logger().info("Logging received parameters...")
        for param, param_value in parameters_dict.items():

            self.get_logger().info(f"{param}: {param_value}")

        self.get_logger().info("Logging parameters done...")

    def parameters_callback(self, params):
        """
            sets the parameters attribute given a 

        Args:
            params (tuple or list of tuples): tuples contain the parameter's name and optionally its value

        Returns:
            set_result(SetParametersResult): an object indicating the result of setting a parameter
        """

        for param in params:
            name = vars(param)["_name"]
            value = vars(param)["_value"]
            setattr(self.PARAM, name, value)

        self.parameters_dict = {
            i: self.get_parameter(i).get_parameter_value().double_value
            for i in PARAMETERS
        }

        self.log_parameters(self.parameters_dict)

        return SetParametersResult(successful=True)

    def timer_callback(self):
        """
            The main tick function that updates the thrusters' values based on the latest
            data from the sensors and cmd_vel
        """

        """
            updates the desired velocities in the x and y axis and the desired depth using the 
            proportional constants  
        """
        v_x = self.desired.v_x * self.PARAM.kp_x
        v_y = self.desired.v_y * self.PARAM.kp_y
        d_z = self.desired.depth * self.PARAM.kp_depth
        """calculates the angular velocity around the z axis (the yaw)
        """
        w_z = self.pid_wz.compute(self.desired.w_z, self.actual.w_z)

    ### logic for rotation
        if self.is_rotating:
            yaw_error = (self.desired.yaw - self.actual.yaw + 180) % 360 - 180  # Normalize to [-180, 180]

        if self.rotation_stage == 0:  # First 360° rotation
            if abs(yaw_error) > 2:  # If yaw error is significant, keep rotating
                self.desired.w_z = 1  # Maintain rotation
            else:
                self.desired.w_z = 0  # Stop rotation
                self.rotation_stage = 1  # Move to next stage
                self.desired.yaw = (self.actual.yaw + 360) % 360  # Set new yaw target
                self.desired.pitch = 45  # Tilt ROV
                self.get_logger().info("First rotation complete! Tilting and rotating again...")

        elif self.rotation_stage == 1:  # Second 360° rotation (with tilt)
            if abs(yaw_error) > 2:
                self.desired.w_z = 1
            else:
                self.desired.w_z = 0
                self.is_rotating = False  # Stop after second rotation
                self.get_logger().info("Second rotation complete! Motion stopped.")
    ###

        """
            calculate the four planner thrusters' values and  bounds them between the minimum and
            maximum value for the thrusters 
        """
        phi_1, phi_2, phi_3, phi_4 = self.ROV.kinematic_control(v_x, v_y, w_z)
        min_phi_val, max_phi_val = self.ROV.find_phi_boudary_values(self.PARAM)

        phi_1, phi_2, phi_3, phi_4 = self.ROV.saturate_actuators(
            phi_1,
            phi_2,
            phi_3,
            phi_4,
            min_phi_val,
            max_phi_val,
        )
        """scales the values of each thruster and add them to a list: planer_thrusters_list 
        """
        phi_1 = map_from_to(
            phi_1,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )
        phi_2 = map_from_to(
            phi_2,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )
        phi_3 = map_from_to(
            phi_3,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )
        phi_4 = map_from_to(
            phi_4,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )

        planer_thrusters_list = [phi_2, phi_1, phi_3, phi_4]

        """
            initialize the distance between the actual depth value and the desired value and
            multiply it with the proportionality constant then scale it between the max and min
            values of the side thrusters 
        """
        depth_error = self.desired.depth - self.actual.depth
        correction_value = depth_error * self.PARAM.kp_depth
        depth_thruster = map_from_to(
            correction_value,
            0,
            2,
            self.PARAM.thruster_side_min,
            self.PARAM.thruster_side_max,
        )

        """
            adjust the side thrusters' values to account for changes in the roll and add them
            to the planer_thrusters_list which is used to populate a Float32MultiArray. 
            the thrusters_voltages message data is logged and then the message is published to
            the  ROV/thrusters topic  
        """
        stabilization_actuation = self.pid_stabilization.compute(
            self.desired.roll, self.actual.roll
        )

        phi_5 = depth_thruster - stabilization_actuation
        phi_6 = depth_thruster + stabilization_actuation

        phi_5 = max(phi_5, self.PARAM.thruster_side_min)
        phi_6 = max(phi_6, self.PARAM.thruster_side_min)
        phi_5 = min(phi_5, self.PARAM.thruster_side_max)
        phi_6 = min(phi_6, self.PARAM.thruster_side_max)

        planer_thrusters_list.append(phi_5)
        planer_thrusters_list.append(phi_6)
        phi_7 = self.pid_pitch.compute(self.actual.pitch, self.desired.pitch)
        phi_7 = max(phi_7, self.PARAM.thruster_side_min)
        phi_7 = min(phi_7, self.PARAM.thruster_side_max)
        planer_thrusters_list.append(phi_7)

        # create the message instance
        thrusters_voltages = Float32MultiArray()

        # fill the message with the phi values
        thrusters_voltages.data = planer_thrusters_list

        # publish the message
        self.get_logger().info(str(list(thrusters_voltages.data)))

        self.thrusters_voltages_publisher.publish(thrusters_voltages)


    ### 
    def rotating_button_callback(self, msg: Bool):
        """callback function for the subscriber to the ROV/rotating topic and updates t self.actual attribute

        Args:
            msg (Bool): boolean sent by the button of rotating
        """
        if msg.data :
            if self.is_rotating:  # to stop it by pressing the button again
                self.desired.w_z=0
                self.is_rotating = False

            else:
            # Start first rotation (360°)
                self.is_rotating = True
                self.rotation_stage = 0  # Start first rotation
                self.desired.yaw = (self.actual.yaw + 360) % 360  # Target yaw
                self.get_logger().info("Rotation Started!")
        
    ###



    def cmd_vel_recieved_callback(self, twist_msg: Twist):
        """
            callback function for the subscriber to the ROV/cmd_vel topic 
            updating the self.desired attribute 

        Args:
            msg (Twist): Twist message sent by the control node
        """
        self.desired.v_x = twist_msg.linear.x
        self.desired.v_y = twist_msg.linear.y
        self.desired.depth = twist_msg.linear.z
        self.desired.w_z = twist_msg.angular.z

    def pitch_received_callback(self, pitch_msg: Float64):
        """updates the target pitch

        Args:
            pitch_msg (Float64): pitch in degrees
        """
        self.get_logger().info(f"Target pitch: {pitch_msg.data}")
        self.desired.pitch = pitch_msg.data
    def depth_recieved_callback(self, depth_msg: Float64):
        """callback function for the subscriber to the ROV/depth topic and updates teh self.actual attribute

        Args:
            msg (Float64): depth sent by the the depth sensor
        """

        self.get_logger().info(f"depth: {depth_msg}")
        self.actual.depth = depth_msg.data

    def imu_recieved_callback(self, imu: Imu):
        """recieves the imu readings from the MPU9025 and updates the self.actual attribute
        Args:
            imu (sensor_msgs/Imu): sensor message containing linear accelerations and angular velocities
        """
        _roll_dot, _pitch_dot, yaw_dot = [
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
        ]
        roll, pitch, yaw = euler_from_quaternion(
            [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        )

        self.actual.w_z = yaw_dot * -1
        self.actual.yaw = yaw * 180 / math.pi
        self.actual.roll = roll * 180 / math.pi
        self.actual.pitch = pitch * 180 / math.pi

        self.get_logger().info(
            f"actual w_z: {self.actual.w_z}, roll: {self.actual.roll}, yaw: {self.actual.yaw}"
        )  # Check degree or radian

    def stop_all(self):
        thrusters_voltages = Float32MultiArray()
        thrusters_voltages.data = [1485, 1485, 1485, 1485, 1485, 1485]
        self.thrusters_voltages_publisher.publish(thrusters_voltages)


def main(args=None):
    rclpy.init(args=args)

    motion_control_node = CalibrationNode()
    rclpy.spin(motion_control_node)
    motion_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()