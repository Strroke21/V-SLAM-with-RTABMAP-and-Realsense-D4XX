import rclpy
from rclpy.node import Node
import time
from transformations import euler_from_quaternion
from pymavlink import mavutil
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import sys

reset_counter = 1
jump_threshold = 0.5 # in meters, from trials and errors
jump_speed_threshold = 20 # in m/s
start_time = time.time()
rng_alt = 0
initial_roll = -3.14159  
conn_string = 'tcp:127.0.0.1:5763'
home_lat = 19.1345054 
home_lon =  72.9120648
home_alt = 53

def normalize_roll(current_roll, initial_roll):
    # Make yaw positive for clockwise rotation (left-to-right)
    roll = -(current_roll - initial_roll)
    # Normalize to [-pi, pi]
    roll = (roll + math.pi) % (2 * math.pi) - math.pi
    return roll

def get_relative_roll(roll):
    global initial_roll
    # Set initial yaw once
    if initial_roll is None:
        initial_roll = roll

    # Get relative yaw: right turn = positive
    relative_roll = normalize_roll(roll, initial_roll)
    return relative_roll


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

def vision_position_send(vehicle, x, y, z, roll, pitch, yaw):

    msg = vehicle.mav.vision_position_estimate_encode(
        int(time.time() * 1e6),
        x, y, z,
        roll, pitch, yaw  
    )
    vehicle.mav.send(msg)

def vision_speed_send(vehicle, vx, vy, vz):

    msg = vehicle.mav.vision_speed_estimate_encode(
        int(time.time() * 1e6),
        vx, vy, vz
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

def increment_reset_counter():
    global reset_counter
    if reset_counter >= 255:
        reset_counter = 1
    reset_counter += 1

def vision_position_delta_send(vehicle, prev_pos, prev_att, curr_pos, curr_att, dt_usec):
    # Compute delta position
    dx = curr_pos[0] - prev_pos[0]
    dy = curr_pos[1] - prev_pos[1]
    dz = curr_pos[2] - prev_pos[2]

    # Compute delta orientation (simplified)
    # Use roll-pitch-yaw difference between current and previous quaternions
    roll1 = prev_att[0]
    pitch1 = prev_att[1]
    yaw1 = prev_att[2]

    roll2 = curr_att[0]
    pitch2 = curr_att[1]
    yaw2 = curr_att[2]

    droll = roll2 - roll1
    dpitch = pitch2 - pitch1
    dyaw = yaw2 - yaw1

    # Normalize angles to [-pi, pi]
    droll = (droll + np.pi) % (2 * np.pi) - np.pi
    dpitch = (dpitch + np.pi) % (2 * np.pi) - np.pi
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    delta_magnitude = np.linalg.norm([dx,dy,dz])  # √(dx² + dy² + dz²)
    confidence = max(0.0, min(90.0, 90.0 - delta_magnitude * 90.0)) #confidence scaled to 90%
    print(f"[Confidence]: {int(confidence)}")
    msg = vehicle.mav.vision_position_delta_encode(
        int(time.time() * 1e6),  # time_usec
        dt_usec,                 # time_delta_usec
        [droll, dpitch, dyaw],   # delta angles in radians
        [dx, dy, dz],            # delta position in meters
        int(confidence) # confidence in percentage
        )
    vehicle.mav.send(msg) #delta position and orientation update

def get_rangefinder_data(vehicle):
    global rng_alt
    # Wait for a DISTANCE_SENSOR or RANGEFINDER message
    msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg is not None:
        dist = msg.current_distance # in meters
        if dist is not None:
            rng_alt = dist/100

    return rng_alt

class SlamLocalization(Node):
    def __init__(self,vehicle):
        super().__init__('localization')
        self.prev_position = None
        self.prev_vel = None
        self.counter = 0
        qos = QoSProfile(depth=0, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(Odometry,'/rtabmap/odom', self.odom_callback, qos)
        self.vehicle = vehicle
        self.counter = 0
        self.prev_pos = None
        self.prev_att = None
        self.prev_time = None
        self.prev_vel = None

    def odom_callback(self, msg):
        linear_vel = msg.twist.twist.linear
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        attitude = euler_from_quaternion(q)
        cam_x, cam_y = position.x, -position.y
        cam_z = -get_rangefinder_data(self.vehicle) # Adjusted for forward facing camera
        cam_vx, cam_vy, cam_vz = linear_vel.x, -linear_vel.y, -linear_vel.z  # Adjusted for forward facing camera
        roll = attitude[2]  
        cam_roll = get_relative_roll(roll)
        cam_pitch = attitude[1]
        cam_yaw = -attitude[0]

        curr_pos = [cam_x, cam_y, cam_z]
        curr_att = [cam_roll, cam_pitch, cam_yaw]
        curr_vel = [cam_vx, cam_vy, cam_vz]
        self.counter += 1
        current_time = time.time()

        vision_position_send(self.vehicle, cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw)
        vision_speed_send(self.vehicle, cam_vx, cam_vy, cam_vz)

        if self.prev_pos is not None and self.prev_att is not None:
            dt_usec = int((current_time - self.prev_time) * 1e6)
            vision_position_delta_send(self.vehicle, self.prev_pos, self.prev_att, curr_pos, curr_att, dt_usec)

            delta_position = [curr_pos[0] - self.prev_pos[0], curr_pos[1] - self.prev_pos[1], curr_pos[2] - self.prev_pos[2]]
            delta_velocity = [curr_vel[0] - self.prev_vel[0], curr_vel[1] - self.prev_vel[1], curr_vel[2] - self.prev_vel[2]]
            position_displacement = np.linalg.norm(delta_position)
            delta_speed = np.linalg.norm(delta_velocity)
            self.get_logger().info(f'[Position Displacement]: {position_displacement:.2f} m, [Speed Delta]: {delta_speed:.2f} m/s')

            if position_displacement > jump_threshold or delta_speed > jump_speed_threshold:

                if position_displacement > jump_threshold:
                    self.get_logger().warn(f'Position jump detected: {position_displacement:.2f} m')
                
                elif delta_speed > jump_speed_threshold:
                    self.get_logger().warn(f'Speed jump detected: {delta_speed:.2f} m/s')

                increment_reset_counter()

        self.get_logger().info(f'[Orientation]: roll: {cam_roll:.2f}, pitch: {cam_pitch:.2f}, yaw: {cam_yaw:.2f}')
        self.get_logger().info(f'[SLAM]: X: {cam_x:.2f}, Y: {cam_y:.2f}, Z: {cam_z:.2f}')  
        self.get_logger().info(f'[Linear Velocity]: x: {cam_vx:.2f}, y: {cam_vy:.2f}, z: {cam_vz:.2f}')
        self.counter += 1
        current_time = time.time()
        data_hz_per_second = self.counter / (current_time - start_time)
        self.get_logger().info(f'Sending to FCU {data_hz_per_second:.2f} Hz')

        self.prev_pos = curr_pos
        self.prev_att = curr_att
        self.prev_time = current_time
        self.prev_vel = curr_vel

    
def main(args=None):
    rclpy.init(args=args)
    vehicle = connect(conn_string,baud=115200)
    enable_data_stream(vehicle, 200)
    set_default_home_position(vehicle, home_lat, home_lon, home_alt)
    time.sleep(1)  
    localization = SlamLocalization(vehicle)
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

