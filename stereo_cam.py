#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class DualCameraPublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_publisher')

        # Publishers for two cameras (image + camera info)
        self.publisher_cam1 = self.create_publisher(Image, 'camera1/image_raw', 10)
        self.publisher_cam2 = self.create_publisher(Image, 'camera2/image_raw', 10)
        self.info_pub_cam1 = self.create_publisher(CameraInfo, 'camera1/camera_info', 10)
        self.info_pub_cam2 = self.create_publisher(CameraInfo, 'camera2/camera_info', 10)

        # OpenCV video captures
        self.cap1 = cv2.VideoCapture(8)  # left camera
        self.cap2 = cv2.VideoCapture(6)  # right camera

        # Force resolution 640x480
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap1.isOpened():
            self.get_logger().error("Failed to open camera 8 (/dev/video8)")
        if not self.cap2.isOpened():
            self.get_logger().error("Failed to open camera 6 (/dev/video6)")

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def make_camera_info(self, frame_id: str, width: int, height: int, is_right=False) -> CameraInfo:
        cam_info = CameraInfo()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = frame_id
        cam_info.height = height
        cam_info.width = width
        cam_info.distortion_model = "plumb_bob"

        if not is_right:
            # Left camera (#8)
            cam_info.d = [-0.22304792, 0.0816238, -0.0043557, 0.01058259, -0.01685162]
            cam_info.k = [
                229.01650254, 0.0, 316.10468785,
                0.0, 227.07382311, 253.98961502,
                0.0, 0.0, 1.0
            ]
        else:
            # Right camera (#6)
            cam_info.d = [-0.22304792, 0.0816238, -0.0043557, 0.01058259, -0.01685162]
            cam_info.k = [
                229.01650254, 0.0, 316.10468785,
                0.0, 227.07382311, 253.98961502,
                0.0, 0.0, 1.0
            ]

        # No rotation (identity)
        cam_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (add baseline for right camera)
        baseline = 0.22  # meters
        fx = cam_info.k[0]
        Tx = -fx * baseline if is_right else 0.0

        cam_info.p = [
            fx, 0.0, cam_info.k[2], Tx,
            0.0, cam_info.k[4], cam_info.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        cam_info.binning_x = 0
        cam_info.binning_y = 0
        cam_info.roi.x_offset = 0
        cam_info.roi.y_offset = 0
        cam_info.roi.height = 0
        cam_info.roi.width = 0
        cam_info.roi.do_rectify = False

        return cam_info

    def timer_callback(self):
        # Camera 1 (left #8)
        ret1, frame1 = self.cap1.read()
        if ret1:
            frame1 = cv2.resize(frame1, (640, 480))
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            msg1 = self.bridge.cv2_to_imgmsg(gray1, encoding="mono8")
            msg1.header.stamp = self.get_clock().now().to_msg()
            msg1.header.frame_id = "camera_infra1_optical_frame"
            self.publisher_cam1.publish(msg1)

            cam_info1 = self.make_camera_info("camera_infra1_optical_frame", 640, 480, is_right=False)
            self.info_pub_cam1.publish(cam_info1)

        # Camera 2 (right #6)
        ret2, frame2 = self.cap2.read()
        if ret2:
            frame2 = cv2.resize(frame2, (640, 480))
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            msg2 = self.bridge.cv2_to_imgmsg(gray2, encoding="mono8")
            msg2.header.stamp = self.get_clock().now().to_msg()
            msg2.header.frame_id = "camera_infra2_optical_frame"
            self.publisher_cam2.publish(msg2)

            cam_info2 = self.make_camera_info("camera_infra2_optical_frame", 640, 480, is_right=True)
            self.info_pub_cam2.publish(cam_info2)

    def destroy_node(self):
        self.cap1.release()
        self.cap2.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
