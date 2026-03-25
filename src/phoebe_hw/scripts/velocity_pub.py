#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")
        self.publisher_ = self.create_publisher(
            Twist, "/r100_0599/platform/cmd_vel_unstamped", 10
        )
        self.timer_period = 0.1  # 10 Hz
        self.running_time = 3.0  # Run for 3 seconds
        self.start_time = time.time()

        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.01
        self.twist_msg.angular.z = 0.0

        self.timer = self.create_timer(self.timer_period, self.publish_velocity)

    def publish_velocity(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time < self.running_time:
            self.publisher_.publish(self.twist_msg)
            self.get_logger().info(f"Publishing: x={self.twist_msg.linear.x}")
        else:
            # Stop the robot after 3 seconds
            self.twist_msg.linear.x = 0.0
            self.publisher_.publish(self.twist_msg)
            self.get_logger().info("Publishing: x=0.0 (Stopping)")
            self.timer.cancel()  # Stop the timer


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
