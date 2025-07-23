import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from pymavlink import mavutil
import time

rng_alt = 0
fcu_addr = '/dev/ttyACM0'  

def connect(connection_string):

    vehicle =  mavutil.mavlink_connection(connection_string)

    return vehicle

def set_parameter(vehicle, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    # Send PARAM_SET message to change the parameter
    vehicle.mav.param_set_send(vehicle.target_system,vehicle.target_component,param_name.encode('utf-8'),param_value,param_type)
    #usage set_parameter(vehicle, "PARAM_NAME", 1)

def send_optical_flow(vehicle, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, ground_distance):
    msg = vehicle.mav.optical_flow_encode(
            int(time.time() * 1e6),  # time_usec
            0,
            flow_x,
            flow_y,
            flow_comp_m_x,
            flow_comp_m_y,
            quality,
            ground_distance,
            0, 0  # flow_rate_x/y rad/s if using gyro
        )
    vehicle.mav.send(msg)

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def get_rangefinder_data(vehicle):
    global rng_alt
    while True:
        msg = vehicle.recv_match(type='DISTANCE_SENSOR', blocking=False)
        if msg is not None:
            rng_alt = msg.current_distance/100  # in meters
        return rng_alt

class OpticalFlowNode(Node):
    def __init__(self,vehicle):
        super().__init__('optical_flow_node')
        self.vehicle = vehicle
        self.sub_image = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_time = None
        self.focal_px = 525  # default if not received yet
        self.ground_distance = get_rangefinder_data(vehicle)
        self.sensor_id = 0

    def info_callback(self, msg):
        self.focal_px = msg.k[0]  # fx
        self.destroy_subscription(self.sub_info)  # properly stop after getting intrinsics


    def image_callback(self, msg):
        curr_time = time.time()
        if self.prev_time is None:
            self.prev_time = curr_time
            return

        dt = curr_time - self.prev_time
        if dt == 0: return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        gray = cv2.resize(frame, (320, 240))

        if self.prev_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None,
                                                0.5, 3, 15, 3, 5, 1.2, 0)
            # Mean optical flow in camera frame
            fx = np.mean(flow[..., 0])  # X-axis in image (right)
            fy = np.mean(flow[..., 1])  # Y-axis in image (down)

            # --- Transform raw pixel shift to drone body frame (dpix) ---
            # Camera X → Body Y, Camera Y → -Body X (down-facing camera)
            flow_x = int(-fy)  # Forward (Body X)
            flow_y = int(-fx)  # Right   (Body Y)

            # --- Convert to meters/second (compensated) ---
            # Apply same transformation to metric flow
            flow_comp_m_x = (-fy * self.ground_distance) / (self.focal_px * dt)  # Forward (Body X)
            flow_comp_m_y = (-fx * self.ground_distance) / (self.focal_px * dt)  # Right   (Body Y)


            magnitude = np.linalg.norm(flow, axis=2)
            quality = int(np.clip(np.mean(magnitude) * 10, 0, 255))
            send_optical_flow(self.vehicle, flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality, self.ground_distance)  
            #self.send_mavlink(flow_x, flow_y, flow_comp_m_x, flow_comp_m_y, quality)
            print(f"fx: {fx}, fy: {fy}, flowcomp_mx: {flow_comp_m_x}, flowcomp_my: {flow_comp_m_y}")

        self.prev_gray = gray
        self.prev_time = curr_time


def main():
    vehicle = connect(fcu_addr)
    enable_data_stream(vehicle, 200)
    set_parameter(vehicle, 'FLOW_TYPE', 5)  # mavlink
    set_parameter(vehicle, 'EK3_SRC1_POSXY',0)  # none
    set_parameter(vehicle, 'EK3_SRC1_VELXY', 5)  # optical flow
    set_parameter(vehicle, 'EK3_SRC1_VELZ', 0)  # none
    set_parameter(vehicle, 'EK3_SRC1_POSZ', 1)  #baro
    set_parameter(vehicle, 'EK3_SRC1_YAW', 1)  #compass
    rclpy.init()
    node = OpticalFlowNode(vehicle)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
