import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class ZeroOdomPublisher(Node):
    def __init__(self):
        super().__init__("zero_odom_publisher")
        self.publisher_ = self.create_publisher(Odometry, "/odom", 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz publishing rate

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # All positions and velocities set to zero
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(w=1.0)  # Identity quaternion

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.publisher_.publish(odom_msg)
        self.get_logger().info("Published zero odometry")


def main(args=None):
    rclpy.init(args=args)
    node = ZeroOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
