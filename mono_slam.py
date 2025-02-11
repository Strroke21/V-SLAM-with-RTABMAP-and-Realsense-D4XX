import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from openvino.runtime import Core
import pyrealsense2 as rs
import math

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_rgb = self.create_publisher(Image, 'camera/rgb', 10)
        self.publisher_depth = self.create_publisher(Image, 'camera/depth', 10)
        self.publisher_imu = self.create_publisher(Imu, 'camera/imu', 10)
        self.publisher_camera_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.bridge = CvBridge()

        # Load OpenVINO Inference Engine
        ie = Core()
        model_xml = "/home/deathstroke/Desktop/vision_safe_landing/openvino_midas_v21_small_256.xml"
        model_bin = "/home/deathstroke/Desktop/vision_safe_landing/openvino_midas_v21_small_256.bin"
        network = ie.read_model(model=model_xml, weights=model_bin)
        self.compiled_model = ie.compile_model(network, "CPU")

        # Get input and output tensor names
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)

        # Open video capture
        self.cap = cv2.VideoCapture(4)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Error opening video file")
            rclpy.shutdown()

        # Configure RealSense IMU pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        self.pipeline.start(config)

        # Create timers
        self.timer = self.create_timer(0.1, self.publish_frames)
        self.imu_timer = self.create_timer(0.1, self.publish_imu)
        self.camera_info_timer = self.create_timer(0.1, self.publish_camera_info)

    def publish_frames(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Reached the end of the video or failed to read the frame.")
            rclpy.shutdown()
            return

        frame = cv2.resize(frame, (640, 480))
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (256, 256)).astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)[np.newaxis, ...]

        # Perform inference
        result = self.compiled_model([img])[self.output_layer]

        # Process output depth map
        depth_map = result.squeeze()
        depth_map = cv2.resize(depth_map, (640, 480), interpolation=cv2.INTER_CUBIC)

        # Normalize depth values (ensure it's in meters)
        depth_map = depth_map.astype(np.float32)
        depth_map = depth_map / np.max(depth_map)  # Normalize between 0 and 1

        # Convert images to ROS 2 messages
        msg_rgb = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg_depth = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")  # Ensure 32-bit float format

        # Publish messages
        self.publisher_rgb.publish(msg_rgb)
        self.publisher_depth.publish(msg_depth)
        self.get_logger().info("Published RGB and Depth images in correct formats")

    def publish_imu(self):
        frames = self.pipeline.wait_for_frames()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if accel_frame and gyro_frame:
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = accel_frame.as_motion_frame().get_motion_data().x
            imu_msg.linear_acceleration.y = accel_frame.as_motion_frame().get_motion_data().y
            imu_msg.linear_acceleration.z = accel_frame.as_motion_frame().get_motion_data().z

            imu_msg.angular_velocity.x = gyro_frame.as_motion_frame().get_motion_data().x
            imu_msg.angular_velocity.y = gyro_frame.as_motion_frame().get_motion_data().y
            imu_msg.angular_velocity.z = gyro_frame.as_motion_frame().get_motion_data().z

            self.publisher_imu.publish(imu_msg)
            self.get_logger().info("Published IMU data")

    def publish_camera_info(self):
        hfov = 87  # Horizontal Field of View in degrees
        vfov = 58  # Vertical Field of View in degrees
        width = 640  # Image width
        height = 480  # Image height

        # Convert degrees to radians
        hfov_rad = math.radians(hfov)
        vfov_rad = math.radians(vfov)

        # Compute focal lengths using fx = (width / 2) / tan(hfov / 2)
        fx = (width / 2) / math.tan(hfov_rad / 2)
        fy = (height / 2) / math.tan(vfov_rad / 2)

        # Principal points (assume at image center)
        cx = width / 2
        cy = height / 2

        # Create CameraInfo message
        camera_info_msg = CameraInfo()
        camera_info_msg.height = height
        camera_info_msg.width = width
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

        # Intrinsic camera matrix (K)
        camera_info_msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # Rectification matrix (identity for monocular camera)
        camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (P)
        camera_info_msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Publish CameraInfo
        self.publisher_camera_info.publish(camera_info_msg)
        self.get_logger().info("Published CameraInfo")

    def on_shutdown(self):
        self.cap.release()
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

