import rclpy
from rclpy.node import Node
import time
from transformations import euler_from_quaternion
from pymavlink import mavutil
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import sys

os.environ["MAVLINK20"] = "1"

reset_counter = 1
jump_threshold = 0.1 # in meters, from trials and errors
jump_speed_threshold = 20 # in m/s
fcu_addr = '/dev/ttyACM3'  
fcu_baud = 115200
start_time = time.time()
home_lat = 19.1345054
home_lon =  72.9120648
home_alt = 53

def set_default_home_position(vehicle, home_lat, home_lon, home_alt):
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    vehicle.mav.set_home_position_send(
        1,
        int(home_lat * 1e7), 
        int(home_lon * 1e7),
        int(home_alt),
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )

def vision_position_send(vehicle, x, y, z, roll, pitch, yaw, cov, reset_counter):

    msg = vehicle.mav.vision_position_estimate_encode(
        int(time.time() * 1e6),
        x, y, z,
        roll, pitch, yaw,
        cov,
        reset_counter    
    )
    vehicle.mav.send(msg)

def vision_speed_send(vehicle, vx, vy, vz, cov,reset_counter):

    msg = vehicle.mav.vision_speed_estimate_encode(
        int(time.time() * 1e6),
        vx, vy, vz,
        cov,
        reset_counter
        )
    vehicle.mav.send(msg)

def connect(connection_string, baud):

    vehicle = mavutil.mavlink_connection(connection_string,baud)
    
    return vehicle

def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

def send_msg_to_gcs(vehicle,text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'D455: ' + text_to_be_sent
    vehicle.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def forward_telemetry(vehicle, gcs_addr):
    gcs = mavutil.mavlink_connection(gcs_addr)
    msg = vehicle.recv_match(blocking=True)
    gcs.mav.send(msg)

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
        self.prev_vel = None
        self.counter = 0
        qos = QoSProfile(depth=0, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(Odometry,'/rtabmap/odom', self.odom_callback, qos)


    def odom_callback(self, msg):
        linear_vel = msg.twist.twist.linear
        vel_cov = msg.twist.covariance
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pos_cov = msg.pose.covariance
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.get_logger().info(f'[Orientation]: roll: {roll}, pitch: {pitch}, yaw: {yaw}')
        self.get_logger().info(f'[Position data]: {position.x}, {position.y}, {position.z}')
        self.get_logger().info(f'[Linear Velocity]: x: {linear_vel.x}, y: {linear_vel.y}, z: {linear_vel.z}')
        vision_position_send(self.vehicle, position.x, position.y, position.z, roll, pitch, yaw,pos_cov,reset_counter)
        vision_speed_send(self.vehicle, linear_vel.x, linear_vel.y, linear_vel.z, vel_cov, reset_counter)
        if self.prev_position is not None:
            delta_translation = [position.x - self.prev_position.x, position.y - self.prev_position.y, position.z - self.prev_position.z]
            position_displacement = np.linalg.norm(delta_translation)
            if position_displacement > jump_threshold:
                self.get_logger().warn(f'Position jump detected: {position_displacement} m, resetting reset_counter.')
                increment_reset_counter()

        if self.prev_vel is not None:
            delta_velocity = [linear_vel.x - self.prev_vel.x, linear_vel.y - self.prev_vel.y, linear_vel.z - self.prev_vel.z]
            speed_delta = np.linalg.norm(delta_velocity)
            if speed_delta > jump_speed_threshold:
                self.get_logger().warn(f'Speed jump detected: {speed_delta} m/s, resetting reset_counter.')
                increment_reset_counter()

        self.prev_position = position
        self.prev_vel = linear_vel
        self.counter += 1
        current_time = time.time()
        data_hz_per_second = self.counter / (current_time - start_time)
        self.get_logger().info(f'Sending to FCU {data_hz_per_second:.2f} Hz')
        

def main(args=None):
    rclpy.init(args=args)
    vehicle = connect(fcu_addr,baud=fcu_baud) 
    enable_data_stream(vehicle, 100)
    time.sleep(1)  
    send_msg_to_gcs(vehicle, "Slam localization node started")
    set_default_home_position(vehicle, home_lat, home_lon, home_alt)
    localization = SlamLocalization(vehicle)
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()