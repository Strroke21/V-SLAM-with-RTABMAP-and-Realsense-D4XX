#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import time
from pymavlink import mavutil
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import subprocess
import math
import transformations as tf
import sys
import argparse
import shlex

os.environ["MAVLINK20"] = "1"

# ----------------------- CONFIGURATION -----------------------
FCU_ADDR = '/dev/ttyACM0'
FCU_BAUD = 115200
HOME_LAT = 19.1345054
HOME_LON = 72.9120648
HOME_ALT = 53
RANGEFINDER_THRESHOLD = 0.1
RTABMAP_LAUNCH_DELAY = 5
STREAM_RATE = 100

debug_enable = 1  # Set to 1 to enable debug messages

# ----------------------- GLOBAL VARIABLES -----------------------
rng_alt = 0
rtabmap_started = False
compass_enabled = 0 # Set to 1 to enable compass heading correction, 0 to disable
camera_orientation = 0 # 0: forward, 1: downfacing, 2: 45degree (tilted down) forward
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized,
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
use_rangefinder = 0  # Set to 1 to use rangefinder data for altitude correction
use_compass = 0    # Set to 1 to use compass data for heading correction

if camera_orientation == 0:
    H_aeroRef_camRef = np.array([
    [1,  0,  0, 0],   # X forward --> forward
    [0, -1,  0, 0],   # Y left --> right
    [0,  0, -1, 0],   # Z up --> down
    [0,  0,  0, 1]
    ])
    H_cambody_aeroBody = H_aeroRef_camRef

if camera_orientation == 1:  # downfacing (90° pitch down)
    H_aeroRef_camRef = np.array([
    [0,  0, 1, 0],   
    [0,  -1,  0, 0],  
    [1, 0,  0,  0],   
    [0,  0,  0, 1]
    ])
    H_cambody_aeroBody = H_aeroRef_camRef

elif camera_orientation == 2: # 45 degree tilted down forward
    c = math.sqrt(2)/2
    H_aeroRef_camRef = np.array([
    [c, 0, c, 0],   # X forward --> forward
    [0, -1, 0, 0],   # Y left --> right
    [c, 0, -c, 0],   # Z up --> down
    [0,  0,  0, 1]
    ])
    H_cambody_aeroBody = np.linalg.inv(H_aeroRef_camRef)

else:
    H_aeroRef_camRef = np.array([
    [1,  0,  0, 0],   # X forward --> forward
    [0, -1,  0, 0],   # Y left --> right
    [0,  0, -1, 0],   # Z up --> down
    [0,  0,  0, 1]
    ])
    H_cambody_aeroBody = H_aeroRef_camRef

# ----------------------- MAVLINK HELPER FUNCTIONS -----------------------
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

def connect_vehicle(connection_string, baud):
    vehicle = mavutil.mavlink_connection(connection_string, baud)
    vehicle.wait_heartbeat()
    return vehicle

def enable_data_stream(vehicle, stream_rate):
    vehicle.mav.request_data_stream_send(vehicle.target_system,
                                        vehicle.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                        stream_rate, 1)

def get_rangefinder_data(vehicle):
    global rng_alt
    msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=False)
    if msg and msg.current_distance is not None:
        rng_alt = msg.current_distance / 100.0
    return rng_alt

def set_default_home_position(vehicle, lat, lon, alt):
    x = y = z = 0
    q = [1, 0, 0, 0]
    approach_x, approach_y, approach_z = 0, 0, 1
    vehicle.mav.set_home_position_send(1,
                                       int(lat*1e7),
                                       int(lon*1e7),
                                       int(alt),
                                       x, y, z, q,
                                       approach_x, approach_y, approach_z)

def VehicleMode(vehicle, mode):
    modes = ["STABILIZE","ACRO","ALT_HOLD","AUTO","GUIDED","LOITER","RTL","CIRCLE","","LAND"]
    mode_id = modes.index(mode) if mode in modes else 12
    vehicle.mav.set_mode_send(vehicle.target_system,
                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                              mode_id)

def get_heading(vehicle):
    msg = vehicle.recv_match(type='AHRS2', blocking=True)
    if msg:
        return msg.yaw


def scaling_factor(true_altitude, est_altitude):
    scale = true_altitude / est_altitude if est_altitude != 0 else 1.0
    return scale

# ----------------------- MAVLINK VISION HELPERS -----------------------
def vision_position_send(vehicle, x, y, z, roll, pitch, yaw):
    msg = vehicle.mav.vision_position_estimate_encode(
        int(time.time()*1e6), x, y, z, roll, pitch, yaw)
    vehicle.mav.send(msg)

def vision_speed_send(vehicle, vx, vy, vz):
    msg = vehicle.mav.vision_speed_estimate_encode(int(time.time()*1e6), vx, vy, vz)
    vehicle.mav.send(msg)

def correct_velocity_and_position(vx, vy, true_altitude, est_altitude, dt, prev_pos):
    scale = true_altitude / est_altitude if est_altitude != 0 else 1.0
    vx_corr = vx * scale
    vy_corr = vy * scale
    pos_x = prev_pos[0] + vx_corr * dt
    pos_y = prev_pos[1] + vy_corr * dt
    return vx_corr, vy_corr, pos_x, pos_y

# ----------------------- SLAM LOCALIZATION NODE -----------------------
class SlamLocalization(Node):
    def __init__(self, vehicle):
        super().__init__('slam_localization')
        qos = QoSProfile(depth=0, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.vehicle = vehicle
        self.odom_subscription = self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, qos)
        self.last_msg = None
        self.prev_pos = None
        self.prev_att = None
        self.prev_time = None
        self.create_timer(0.065, self.timer_callback)
        if use_compass == 1:
            self.initial_compass_yaw = get_heading(vehicle) 

    def odom_callback(self, msg):
        self.last_msg = msg

    def timer_callback(self):
        global rtabmap_started

        msg = self.last_msg
        linear_vel = msg.twist.twist.linear
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # as per rtabmap coordinate system
        q_cam = [q.w, q.x, q.y, q.z] 

        H_camRef_cambody = tf.quaternion_matrix(q_cam)
        H_camRef_cambody[0][3] = position.x
        H_camRef_cambody[1][3] = position.y
        H_camRef_cambody[2][3] = position.z

        # Transform to aeronautic coordinates (body AND reference frame!)
        H_aeroRef_aeroBody = H_aeroRef_camRef.dot(H_camRef_cambody.dot(H_cambody_aeroBody))

        # Calculate GLOBAL XYZ speed (speed from camera is already GLOBAL) as per T265
        V_aeroRef_aeroBody = tf.quaternion_matrix([1,0,0,0])
        V_aeroRef_aeroBody[0][3] = linear_vel.x
        V_aeroRef_aeroBody[1][3] = linear_vel.y
        V_aeroRef_aeroBody[2][3] = linear_vel.z
        V_aeroRef_aeroBody = H_aeroRef_camRef.dot(V_aeroRef_aeroBody)
        
        if use_rangefinder == 1:
            rng_pos_z = -get_rangefinder_data(self.vehicle)
        else:
            rng_pos_z = V_aeroRef_aeroBody[2][3]

        #angles
        rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))
        
        # Realign heading to face north using initial compass data
        if compass_enabled==1:
            H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot( tf.euler_matrix(0, 0, self.initial_compass_yaw, 'szyx'))

        curr_pos = [H_aeroRef_aeroBody[0][3], H_aeroRef_aeroBody[1][3], H_aeroRef_aeroBody[2][3]]
        curr_att = [rpy_rad[0], rpy_rad[1], rpy_rad[2]]
        curr_vel = [V_aeroRef_aeroBody[0][3], V_aeroRef_aeroBody[1][3], V_aeroRef_aeroBody[2][3]]
        curr_time = time.time()

        if self.prev_pos is not None:
            dt_sec = curr_time - self.prev_time
            vx, vy, pos_x, pos_y = correct_velocity_and_position(
                curr_vel[0], curr_vel[1],
                rng_pos_z, H_aeroRef_aeroBody[2][3], dt_sec, self.prev_pos)
            
            print(f"Corrected Velocity: vx={vx}, vy={vy}, vz={curr_vel[2]}")
            print(f"Corrected Position: x={pos_x}, y={pos_y}, z={rng_pos_z}")

            vision_speed_send(self.vehicle, vx, vy, curr_vel[2])
            vision_position_send(self.vehicle, pos_x, pos_y, rng_pos_z,curr_att[0], curr_att[1], curr_att[2])

            # Show debug messages here
            if debug_enable == 1:
                os.system('clear') # This helps in displaying the messages to be more readable
                progress("DEBUG: Raw RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_camRef_cambody, 'sxyz')) * 180 / math.pi))
                progress("DEBUG: NED RPY[deg]: {}".format( np.array(rpy_rad) * 180 / math.pi))
                progress("DEBUG: Raw pos xyz : {}".format( np.array( [position.x, position.y, position.z])))
                progress("DEBUG: NED pos xyz : {}".format( np.array( tf.translation_from_matrix( H_aeroRef_aeroBody))))
                print(f"Scaled Position: x={pos_x}, y={pos_y}, z={rng_pos_z}")
                

        self.prev_pos = curr_pos
        self.prev_att = curr_att        
        self.prev_time = curr_time

# ----------------------- MAIN -----------------------
def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros_cam", type=str, required=True)
    parser.add_argument("--rtabmap", type=str, required=True)

    # FIRST parse arguments
    args = parser.parse_args()

    # THEN split commands
    ros_cam_cmd = shlex.split(args.ros_cam)
    rtabmap_cmd = shlex.split(args.rtabmap)

    # Now safe to use args
    rclpy.init(args=None)

    vehicle = connect_vehicle(FCU_ADDR, FCU_BAUD)
    enable_data_stream(vehicle, STREAM_RATE)
    time.sleep(1)

    set_default_home_position(vehicle, HOME_LAT, HOME_LON, HOME_ALT)

    print("Launching the camera...")
    subprocess.Popen(ros_cam_cmd)

    print("Waiting for camera initialization...")
    time.sleep(20)

    print("Launching RTAB-Map...")
    subprocess.Popen(rtabmap_cmd)

    time.sleep(RTABMAP_LAUNCH_DELAY)

    slam_node = SlamLocalization(vehicle)
    rclpy.spin(slam_node)

    slam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()

