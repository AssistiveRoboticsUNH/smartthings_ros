import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(5.0, self.publish_initial_pose)
        self.publish_count = 0
        self.publish_limit = 4  # Number of times to publish

    def publish_initial_pose(self):
        if self.publish_count < self.publish_limit:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            pose_msg.pose.pose = Pose(
                position=Point(x=1.4583, y=-4.44, z=0.0023),
                orientation=Quaternion(w=1.0)
            )

            # Set covariance to zero
            pose_msg.pose.covariance = [0.0] * 36

            self.publisher.publish(pose_msg)
            self.get_logger().info(f'Published initial pose message {self.publish_count + 1}/{self.publish_limit}.')
            self.publish_count += 1
        else:
            self.get_logger().info('Publishing limit reached. Stopping...')
            self.timer.cancel()  # Stop the timer

def main(args=None):
    rclpy.init(args=args)

    initial_pose_publisher = InitialPosePublisher()

    # Keep the node running until interrupted
    rclpy.spin(initial_pose_publisher)

    # Clean up
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
