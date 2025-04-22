import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import math
import numpy as np

class ROVSimulator(Node):
    def __init__(self):
        super().__init__('rov_simulator')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ROV/thrusters',
            self.listener_callback,
            10)

        self.imu_pub = self.create_publisher(Imu, 'ROV/imu', 10)
        self.depth_pub = self.create_publisher(Float64, 'ROV/depth', 10)
        self.pose_pub = self.create_publisher(Pose, 'ROV/pose', 10)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update_physics)

        self.pos = [0.0, 0.0, 0.0]  # x, y, z
        self.vel = [0.0, 0.0, 0.0]  # velocity x, y, z
        self.yaw = 0.0
        self.w_z = 0.0

        self.last_forces = [1500.0] * 7

        self.mass = 1.0  # kg
        self.inertia_z = 5.0  # moment of inertia around Z axis

        self.thruster_min = 1100
        self.thruster_max = 1900
        self.thruster_side_min = 1300
        self.thruster_side_max = 1700
        self.max_thrust = 1.0  # N

        self.linear_drag_coefficient = 0.1  # N/(m/s)^2
        self.angular_drag_coefficient = 0.05  # Nm/(rad/s)^2

    def pwm_to_force(self, pwm, pwm_min, pwm_max, force_max):
        pwm = max(pwm_min, min(pwm, pwm_max))
        return ((pwm - pwm_min) / (pwm_max - pwm_min)) * 2 * force_max - force_max

    def listener_callback(self, msg):
        self.last_forces = msg.data

    def update_physics(self):
        if len(self.last_forces) < 6:
            return

        sqrt2 = math.sqrt(2) / 2
        vectors = [
            ( sqrt2,  sqrt2),  # T1
            (-sqrt2,  sqrt2),  # T2
            ( sqrt2, -sqrt2),  # T3
            (-sqrt2, -sqrt2),  # T4
        ]
        yaw_signs = [1, -1, -1, 1]

        fx_total, fy_total, torque_z = 0.0, 0.0, 0.0

        for i in range(4):
            f = self.pwm_to_force(self.last_forces[i], self.thruster_min, self.thruster_max, self.max_thrust)
            fx_total += vectors[i][0] * f
            fy_total += vectors[i][1] * f
            torque_z += yaw_signs[i] * f

        # Apply quadratic drag
        fx_total -= self.linear_drag_coefficient * self.vel[0] * abs(self.vel[0])
        fy_total -= self.linear_drag_coefficient * self.vel[1] * abs(self.vel[1])
        torque_z -= self.angular_drag_coefficient * self.w_z * abs(self.w_z)

        # Convert force to acceleration
        ax = fx_total / self.mass
        ay = fy_total / self.mass
        alpha_z = torque_z / self.inertia_z

        # Integrate acceleration to get velocity
        self.vel[0] += ax * self.dt
        self.vel[1] += ay * self.dt
        self.w_z += alpha_z * self.dt

        # Integrate velocity to get position
        self.pos[0] += self.vel[0] * self.dt
        self.pos[1] += self.vel[1] * self.dt
        self.yaw += self.w_z * self.dt

        # Vertical motion
        f5 = self.pwm_to_force(self.last_forces[4], self.thruster_side_min, self.thruster_side_max, self.max_thrust)
        f6 = self.pwm_to_force(self.last_forces[5], self.thruster_side_min, self.thruster_side_max, self.max_thrust)

        fz = (f5 + f6) / 2.0
        fz -= self.linear_drag_coefficient * self.vel[2] * abs(self.vel[2])
        az = fz / self.mass

        self.vel[2] += az * self.dt
        self.pos[2] += self.vel[2] * self.dt

        # IMU message
        imu_msg = Imu()
        imu_msg.orientation.z = math.sin(self.yaw / 2.0)
        imu_msg.orientation.w = math.cos(self.yaw / 2.0)
        imu_msg.angular_velocity.z = self.w_z
        self.imu_pub.publish(imu_msg)

        # Depth message
        depth_msg = Float64()
        depth_msg.data = self.pos[2]
        self.depth_pub.publish(depth_msg)

        # Pose message
        pose_msg = Pose()
        pose_msg.position.x = self.pos[0]
        pose_msg.position.y = self.pos[1]
        pose_msg.position.z = self.pos[2]
        pose_msg.orientation.z = imu_msg.orientation.z
        pose_msg.orientation.w = imu_msg.orientation.w
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROVSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
