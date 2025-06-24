import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from transformations import euler_from_quaternion
from pymavlink import mavutil
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

reset_counter = 1
jump_threshold = 0.1 # in meters, from trials and errors, should be relative to how frequent is the position data obtained


def vision_position_send(vehicle, x, y, z, roll, pitch, yaw):

    vehicle.mav.vision_position_estimate_send(
        int(time.time() * 1e6),
        x, y, z,
        roll, pitch, yaw
    )

def vision_speed_send(vehicle, vx, vy, vz):

    vehicle.mav.vision_speed_estimate_send(
        int(time.time() * 1e6),
        vx, vy, vz
        )


def connect(connection_string):

    vehicle = mavutil.mavlink_connection(connection_string)
    
    return vehicle

def increment_reset_counter():
    global reset_counter
    if reset_counter >= 255:
        reset_counter = 1
    reset_counter += 1

class SlamLocalization(Node):
    def __init__(self, vehicle):
        super().__init__('localization')
        self.vehicle = vehicle
        self.prev_position = None
        # Subscribe to the /rtabmap/localization_pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.localization_pose_callback,
            10
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(Odometry,'/rtabmap/odom', self.odom_callback, qos)

    def localization_pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        cov = msg.pose.covariance  # 6x6 covariance matrix
        roll, pitch, yaw = euler_from_quaternion(q)
        vision_position_send(self.vehicle, position.x, position.y, position.z, roll, pitch, yaw)
        self.get_logger().info(f'Sending vision position estimate to vehicle')
        self.get_logger().info(f'Pos_x: {position.x}, Pos_y: {position.y}, Pos_z: {position.z}, Roll= {roll}, Pitch: {pitch}, Yaw: {yaw}, reset_counter: {reset_counter}')
    
        if self.prev_position is not None:
            delta_translation = [position.x - self.prev_position.x, position.y - self.prev_position.y, position.z - self.prev_position.z]
            position_displacement = np.linalg.norm(delta_translation)
            if position_displacement > jump_threshold:
                self.get_logger().warn(f'Position jump detected: {position_displacement} m, resetting reset_counter.')
                increment_reset_counter()

        self.prev_position = position

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        cov = msg.twist.covariance
        self.get_logger().info(f'Linear Velocity - x: {linear.x}, y: {linear.y}, z: {linear.z}')
        vision_speed_send(self.vehicle, linear.x, linear.y, linear.z)


def main(args=None):
    rclpy.init(args=args)
    # Replace with your actual MAVLink connection string!
    vehicle = connect('10.61.225.90:14550') #replace with your connection string
    localization = SlamLocalization(vehicle)
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()