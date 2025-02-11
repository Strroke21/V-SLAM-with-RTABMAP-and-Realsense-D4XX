#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthImageVisualizer(Node):
    def __init__(self):
        super().__init__('depth_image_visualizer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # The topic name for depth image
            self.depth_image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info("Depth Image Visualizer Node started.")

    def depth_image_callback(self, msg):
        try:
            # Convert the depth image message to an OpenCV image (16-bit)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            # Convert the 16-bit depth image to a format that can be displayed
            # Optional: Normalize the image for better visualization (scale it to 0-255)
            depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_image_normalized = depth_image_normalized.astype('uint8')
            
            # Show the depth image
            cv2.imshow("Depth Image", depth_image_normalized)
            
            # Exit display window on key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    depth_image_visualizer = DepthImageVisualizer()
    try:
        rclpy.spin(depth_image_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        depth_image_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
