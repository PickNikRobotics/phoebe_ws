#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EmergencyStopPublisher(Node):
    def __init__(self):
        super().__init__('emergency_stop_publisher')
        self.publisher_ = self.create_publisher(Bool, '/r100_0599/platform/emergency_stop', 10)
        self.timer = self.create_timer(1.0, self.publish_emergency_stop)

    def publish_emergency_stop(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = EmergencyStopPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
