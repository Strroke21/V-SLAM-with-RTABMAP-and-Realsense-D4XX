#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import time
from transformations import euler_from_quaternion
from pymavlink import mavutil
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import subprocess
import math

os.environ["MAVLINK20"] = "1"

# ----------------------- CONFIGURATION -----------------------
FCU_ADDR = 'tcp:127.0.0.1:5763'
FCU_BAUD = 115200
HOME_LAT = 19.1345054
HOME_LON = 72.9120648
HOME_ALT = 53
RANGEFINDER_THRESHOLD = 5.0
RTABMAP_LAUNCH_DELAY = 5
STREAM_RATE = 100

# ----------------------- GLOBAL VARIABLES -----------------------
rng_alt = 0
initial_yaw = None
rtabmap_started = False

# ----------------------- MAVLINK HELPER FUNCTIONS -----------------------
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

# ----------------------- ORIENTATION & VELOCITY HELPERS -----------------------
def normalize_yaw(current_yaw, initial_yaw):
    yaw = -(current_yaw - initial_yaw)
    yaw = (yaw + math.pi) % (2*math.pi) - math.pi
    return yaw

def get_relative_yaw(orientation):
    global initial_yaw
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    if initial_yaw is None:
        initial_yaw = yaw
    return normalize_yaw(yaw, initial_yaw)

def rotate_to_world(attitude):
    cr = math.cos(attitude[0]); sr = math.sin(attitude[0])
    cp = math.cos(attitude[1]); sp = math.sin(attitude[1])
    cy = math.cos(attitude[2]); sy = math.sin(attitude[2])
    R = [
        [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
        [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
        [-sp, sr*cp, cr*cp]
    ]
    return [math.atan2(R[2][1], R[2][2]),
            math.asin(-R[2][0]),
            math.atan2(R[1][0], R[0][0])]

def correct_velocity_and_position(vx, vy, true_altitude, est_altitude, dt, prev_pos):
    scale = true_altitude / est_altitude if est_altitude != 0 else 1.0
    vx_corr = vx * scale
    vy_corr = vy * scale
    pos_x = prev_pos[0] + vx_corr * dt
    pos_y = prev_pos[1] + vy_corr * dt
    return vx_corr, vy_corr, pos_x, pos_y

# ----------------------- MAVLINK VISION HELPERS -----------------------
def vision_position_send(vehicle, x, y, z, roll, pitch, yaw):
    msg = vehicle.mav.vision_position_estimate_encode(
        int(time.time()*1e6), x, y, z, roll, pitch, yaw)
    vehicle.mav.send(msg)

def vision_speed_send(vehicle, vx, vy, vz):
    msg = vehicle.mav.vision_speed_estimate_encode(int(time.time()*1e6), vx, vy, vz)
    vehicle.mav.send(msg)

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

    def odom_callback(self, msg):
        self.last_msg = msg

    def timer_callback(self):
        global rtabmap_started

        # Launch RTAB-Map if height > threshold
        if not rtabmap_started and get_rangefinder_data(self.vehicle) > RANGEFINDER_THRESHOLD:
            self.get_logger().info(f"Altitude > {RANGEFINDER_THRESHOLD}m, launching RTAB-Map in {RTABMAP_LAUNCH_DELAY}s...")
            time.sleep(RTABMAP_LAUNCH_DELAY)
            subprocess.Popen([
                "ros2", "launch", "rtabmap_launch", "rtabmap.launch.py",
                "rtabmap_args:='--delete_db_on_start'",
                "stereo:=true",
                "left_image_topic:=/zed/zed_node/left_gray/image_rect_gray",
                "right_image_topic:=/zed/zed_node/right_gray/image_rect_gray",
                "left_camera_info_topic:=/zed/zed_node/left_gray/camera_info",
                "right_camera_info_topic:=/zed/zed_node/right_gray/camera_info",
                "frame_id:=zed_left_camera_frame",
                "use_sim_time:=true",
                "approx_sync:=true",
                "qos:=2",
                "rviz:=false",
                "queue_size:=100",
                "imu_topic:=/zed/zed_node/imu/data"
            ])
            rtabmap_started = True

        if self.last_msg is None:
            return

        msg = self.last_msg
        linear_vel = msg.twist.twist.linear
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        attitude = [roll, pitch, yaw]

        cam_x, cam_y = position.z, -position.y
        cam_z = -get_rangefinder_data(self.vehicle)
        cam_vx, cam_vy, cam_vz = linear_vel.z, -linear_vel.y, linear_vel.x
        cam_roll, cam_pitch, cam_yaw = attitude[0], attitude[1], -attitude[2] #camera should be forward initially to remove cam_yaw ambiguity before facing it down

        curr_pos = [cam_x, cam_y, cam_z]
        curr_att = [cam_roll, cam_pitch, cam_yaw]
        curr_time = time.time()

        if self.prev_pos is not None:
            dt_sec = curr_time - self.prev_time
            vx, vy, pos_x, pos_y = correct_velocity_and_position(cam_vx, cam_vy, cam_z, position.x, dt_sec, self.prev_pos)
            vision_position_send(self.vehicle, pos_x, pos_y, cam_z, cam_roll, cam_pitch, cam_yaw)
            vision_speed_send(self.vehicle, vx, vy, cam_vz)
            hz = 1.0/dt_sec if dt_sec>0 else 0
            print(f"x:{pos_x:.2f}, y:{pos_y:.2f}, z:{cam_z:.2f}, Hz:{hz:.2f}")
            print(f"Roll:{cam_roll:.2f}, Pitch:{cam_pitch:.2f}, Yaw:{cam_yaw:.2f}")
            
        self.prev_pos = curr_pos
        self.prev_att = curr_att
        self.prev_time = curr_time

# ----------------------- MAIN -----------------------
def main(args=None):
    rclpy.init(args=args)
    vehicle = connect_vehicle(FCU_ADDR, FCU_BAUD)
    enable_data_stream(vehicle, STREAM_RATE)
    time.sleep(1)
    set_default_home_position(vehicle, HOME_LAT, HOME_LON, HOME_ALT)
    print("Launching ZED2i camera...")
    subprocess.Popen([
        "ros2", "launch", "zed_wrapper", "zed_camera.launch.py",
        "camera_model:=zed2i",
        "enable_ipc:=false",
        'ros_params_override_path:=/home/deathstroke/zed_conf.yaml'
    ])
    print("Waiting 30 seconds for ZED2i initialization...")
    time.sleep(30)
    slam_node = SlamLocalization(vehicle)
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()