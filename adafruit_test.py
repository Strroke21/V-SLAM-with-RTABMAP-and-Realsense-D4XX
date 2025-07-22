import time
import serial
from adafruit_bno08x_rvc import BNO08x_RVC

# Use pyserial to open the UART directly
uart = serial.Serial("/dev/ttyTHS1", baudrate=115200, timeout=1)

# Create BNO08x_RVC instance using raw serial
rvc = BNO08x_RVC(uart)

# Main loop
while True:
    try:
        roll, pitch, yaw, x_accel, y_accel, z_accel = rvc.heading
        print(f"Roll: {roll:.2f}  Pitch: {pitch:.2f}  Yaw: {yaw:.2f} degrees")
        print(f"Acceleration X: {x_accel:.3f}  Y: {y_accel:.3f}  Z: {z_accel:.3f} m/s²\n")
        #time.sleep(0.1)
    except RuntimeError:
        # Skip bad reads
        continue
