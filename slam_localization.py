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
jump_threshold = 0.1 # in meters, from trials and errors


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

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

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
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(Odometry,'/rtabmap/odom', self.odom_callback, qos)


    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        cov = msg.twist.covariance
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.get_logger().info(f'roll: {roll}, pitch: {pitch}, yaw: {yaw}')
        self.get_logger().info(f'Position data: {position.x}, {position.y}, {position.z}')
        self.get_logger().info(f'Linear Velocity - x: {linear.x}, y: {linear.y}, z: {linear.z}')
        vision_position_send(self.vehicle, position.x, position.y, position.z, roll, pitch, yaw)
        vision_speed_send(self.vehicle, linear.x, linear.y, linear.z)
        if self.prev_position is not None:
            delta_translation = [position.x - self.prev_position.x, position.y - self.prev_position.y, position.z - self.prev_position.z]
            position_displacement = np.linalg.norm(delta_translation)
            if position_displacement > jump_threshold:
                self.get_logger().warn(f'Position jump detected: {position_displacement} m, resetting reset_counter.')
                increment_reset_counter()

        self.prev_position = position


def main(args=None):
    rclpy.init(args=args)
    vehicle = connect('udp:10.61.225.90:14550') 
    enable_data_stream(vehicle, 100)
    time.sleep(1)  
    localization = SlamLocalization(vehicle)
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()