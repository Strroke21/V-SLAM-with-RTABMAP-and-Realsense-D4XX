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
import csv
import math

os.environ["MAVLINK20"] = "1"

reset_counter = 1
jump_threshold = 0.1 # in meters, from trials and errors
jump_speed_threshold = 20 # in m/s
fcu_addr = '/dev/ttyACM0'  
fcu_baud = 115200
start_time = time.time()
home_lat = 19.1345054 
home_lon =  72.9120648
home_alt = 53
rng_alt = 0
initial_yaw = -3.14159

#camera downfacing: cam_x = slam_z, cam_y = -slam_y, cam_z = slam_x, cam_roll = slam_yaw, cam_pitch = slam_pitch, cam_yaw = slam_roll
#camera forward: cam_x = slam_x, cam_y = -slam_y, cam_z = -slam_z

def get_rangefinder_data(vehicle):
    global rng_alt
    # Wait for a DISTANCE_SENSOR or RANGEFINDER message
    msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg is not None:
        dist = msg.current_distance # in meters
        if dist is not None:
            rng_alt = dist/100

    return rng_alt

def normalize_yaw(current_yaw, initial_yaw):
    # Make yaw positive for clockwise rotation (left-to-right)
    yaw = -(current_yaw - initial_yaw)

    # Normalize to [-pi, pi]
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    return yaw

def get_relative_yaw(orientation):
    global initial_yaw

    # Quaternion to Euler
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    # Set initial yaw once
    if initial_yaw is None:
        initial_yaw = yaw

    # Get relative yaw: right turn = positive
    relative_yaw = normalize_yaw(yaw, initial_yaw)
    return relative_yaw

def rotate_to_world(attitude):
    # Convert from body frame to NED/world frame
    cr = math.cos(attitude[0])
    sr = math.sin(attitude[0])
    cp = math.cos(attitude[1])
    sp = math.sin(attitude[1])
    cy = math.cos(attitude[2])
    sy = math.sin(attitude[2])

    # Rotation matrix R_body_to_world
    R = [
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp]
    ]

    # Convert attitude to world frame
    return [
        math.atan2(R[2][1], R[2][2]),  # Roll
        math.asin(-R[2][0]),            # Pitch
        math.atan2(R[1][0], R[0][0])    # Yaw
    ]

def get_local_position(vehicle):
    while True:
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg is not None:
            pos_x = msg.x # meters
            pos_y = msg.y  # meters
            pos_z = msg.z  # Meters
            vx = msg.vx
            vy = msg.vy
            vz = msg.vz
            return [pos_x,pos_y,pos_z,vx,vy,vz]
    
def set_parameter(vehicle, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(
        vehicle.target_system,
        vehicle.target_component,
        param_name.encode('utf-8'),
        param_value,
        param_type)
    

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

def forward_telemetry(vehicle, gcs_addr):
    gcs = mavutil.mavlink_connection(gcs_addr)
    msg = vehicle.recv_match(blocking=True)
    gcs.mav.send(msg)

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
    confidence = max(0.0, min(100.0, 100.0 - delta_magnitude * 100.0))
    print(f"Confidence: {confidence:.2f}")
    # Build and send the message
    msg = vehicle.mav.vision_position_delta_encode(
        int(time.time() * 1e6),  # time_usec
        dt_usec,                 # time_delta_usec
        [math.radians(droll), math.radians(dpitch), math.radians(dyaw)],   # delta angles in radians
        [dx, dy, dz],            # delta position in meters
        confidence # confidence in percentage
        )
    vehicle.mav.send(msg)

class SlamLocalization(Node):
    def __init__(self, vehicle):
        super().__init__('localization')
        self.vehicle = vehicle
        self.prev_position = None
        self.prev_vel = None
        self.counter = 0
        qos = QoSProfile(depth=0, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(Odometry,'/rtabmap/odom', self.odom_callback, qos)
        self.csv_file = open('slam_log.csv', mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['SLAM_X', 'SLAM_Y', 'SLAM_Z', 'FUSED_X', 'FUSED_Y', 'FUSED_Z'])
        self.prev_pos = None
        self.prev_att = None
        self.prev_time = None
        self.prev_state = np.array([[0],[0],[0],[0]]) #initialisation state
        self.prev_cov = np.eye(4)*0.025 #initial covariance

    def odom_callback(self, msg):
        linear_vel = msg.twist.twist.linear
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        attitude = [roll, pitch, yaw]
        cam_x, cam_y, _ = position.z, -position.y, position.x  # Adjusted for downfacing camera
        cam_z = -get_rangefinder_data(self.vehicle)  # Use rangefinder data for Z
        cam_vx, cam_vy, cam_vz = linear_vel.z, -linear_vel.y, linear_vel.x  # Adjusted for downfacing camera
        cam_roll, cam_pitch, yaw = rotate_to_world(attitude) # Adjusted for downfacing camera
        cam_yaw = get_relative_yaw(orientation) #relative to body frame

        vision_position_send(self.vehicle, cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw)
        vision_speed_send(self.vehicle, cam_vx, cam_vy, cam_vz)

        self.counter += 1
        current_time = time.time()
        curr_pos = [cam_x, cam_y, cam_z]
        curr_att = [cam_roll, cam_pitch, cam_yaw]

        if self.prev_pos is not None and self.prev_att is not None:
            dt_usec = int((current_time - self.prev_time) * 1e6)
            vision_position_delta_send(self.vehicle, self.prev_pos, self.prev_att, curr_pos, curr_att, dt_usec)

        self.prev_pos = curr_pos
        self.prev_att = curr_att
        self.prev_time = current_time

        data_hz_per_second = self.counter / (current_time - start_time)
        self.get_logger().info(f'Sending to FCU {data_hz_per_second:.2f} Hz')
        self.get_logger().info(f'[Orientation]: roll: {cam_roll}, pitch: {cam_pitch}, yaw: {cam_yaw}')
        self.get_logger().info(f'[SLAM]: X: {cam_x}, Y: {cam_y}, Z: {cam_z}')  
        self.get_logger().info(f'[Linear Velocity]: x: {cam_vx}, y: {cam_vy}, z: {cam_vz}')
        self.csv_writer.writerow([cam_x, cam_y, cam_z])

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()
        self.get_logger().info("CSV log file closed.")
        
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
