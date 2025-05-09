import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class LocalizationPoseSubscriber(Node):
    def __init__(self):
        super().__init__('localization_pose_subscriber')
        # Subscribe to the /rtabmap/localization_pose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.localization_pose_callback,
            10
        )
        self.subscription

    def localization_pose_callback(self, msg):
        # Extract the position (x, y, z) and orientation (quaternion)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Log the position and orientation
        self.get_logger().info(f'Position: x={position.x}, y={position.y}, z={position.z}')
        self.get_logger().info(f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')

def main(args=None):
    rclpy.init(args=args)
    localization_pose_subscriber = LocalizationPoseSubscriber()
    rclpy.spin(localization_pose_subscriber)
    localization_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # main 
    main()
