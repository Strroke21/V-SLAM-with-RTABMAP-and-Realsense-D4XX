

from pymavlink import mavutil
import math
import time


def connect(connection_string, baud):

    vehicle = mavutil.mavlink_connection(connection_string,baud)
    
    return vehicle


def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def send_velocity_setpoint(vehicle, vx, vy, vz):

    # Send MAVLink command to set velocity
    vehicle.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,       # target_system
        vehicle.target_component,    # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,        # type_mask (only vx, vy, vz, yaw_rate)
        0, 0, 0,                    # position (not used)
        vx, vy, vz,                 # velocity in m/s
        0, 0, 0,                    # acceleration (not used)
        0, 0                        # yaw, yaw_rate (not used)
    )


def apply_deadband(value, threshold=50):
    return 0 if abs(value) < threshold else value

def clamp_velocity(v, threshold=0.001):  # m/s
    return 0 if abs(v) < threshold else v


def rotate_to_world(imu, attitude):
    # Convert from body frame to NED/world frame
    cr = math.cos(attitude.roll)
    sr = math.sin(attitude.roll)
    cp = math.cos(attitude.pitch)
    sp = math.sin(attitude.pitch)
    cy = math.cos(attitude.yaw)
    sy = math.sin(attitude.yaw)

    # Rotation matrix R_body_to_world
    R = [
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp]
    ]

    # Raw accel in milli-g → convert to m/s²
    scale = 9.80665 / 1000.0
    ax = imu.xacc * scale
    ay = imu.yacc * scale
    az = imu.zacc * scale

    # Rotate body-frame acceleration to world frame
    ax_world = R[0][0] * ax + R[0][1] * ay + R[0][2] * az
    ay_world = R[1][0] * ax + R[1][1] * ay + R[1][2] * az
    az_world = R[2][0] * ax + R[2][1] * ay + R[2][2] * az

    # Subtract gravity from Z component (gravity is ~9.81 m/s² downward)
    az_world -= 9.80665

    return ax_world, ay_world, az_world


def calculate_position_from_imu(imu_obj, attitude):
    global previous_time, vx, vy, x_total, y_total

    if imu_obj is None or attitude is None:
        return x_total, y_total, 0.0

    current_time = imu_obj.time_boot_ms / 1000.0
    if previous_time is None:
        previous_time = current_time
        return x_total, y_total, 0.0

    dt = current_time - previous_time
    previous_time = current_time

    # Rotate + scale + gravity correct
    ax_w, ay_w, az_w = rotate_to_world(imu_obj, attitude)

    # Integrate acceleration → velocity
    vx += ax_w * dt
    vy += ay_w * dt

    # Optional: decay and clamp
    # vx *= 0.98
    # vy *= 0.98
    vx = clamp_velocity(vx)
    vy = clamp_velocity(vy)
    # Remove this, or:
    vx = clamp_velocity(vx, 1e-6)
    vy = clamp_velocity(vy, 1e-6)

    # Integrate velocity → position
    x_total += vx * dt
    y_total += vy * dt

    print(f"ax_world={ax_w:.3f}, ay_world={ay_w:.3f}, vx={vx:.3f}, vy={vy:.3f}, dt={dt:.4f}")
    return x_total, y_total, dt

    
previous_time = None
vx, vy = 0.0, 0.0
x_total, y_total = 0.0, 0.0
vehicle = connect('tcp:127.0.0.1:5763', 115200) #'/dev/ttyACM0'

enable_data_stream(vehicle, 100)
counter = 0
start_time = time.time()

while True:
    curr_time = time.time()
    imu = vehicle.recv_match(type='SCALED_IMU2', blocking=True)
    ned = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    attitude = vehicle.recv_match(type='ATTITUDE',blocking=True)
    print(ned)
    x,y,dt = calculate_position_from_imu(imu, attitude)
    print(f"calculated Pos: x: {x} y: {y} dt: {dt}")
    counter += 1
    total_time = int(curr_time - start_time)
    if time.time() - start_time < 300:
        send_velocity_setpoint(vehicle, 10,0,0)

    if total_time > 299:
        break

        


        