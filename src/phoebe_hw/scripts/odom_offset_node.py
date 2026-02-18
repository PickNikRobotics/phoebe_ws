#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Trigger

import copy
from scipy.spatial.transform import Rotation as R


def quaternion_multiply(q1, q2):
    return (R.from_quat(q1) * R.from_quat(q2)).as_quat()


def quaternion_inverse(q):
    return R.from_quat(q).inv().as_quat()


class OdomOffsetNode(Node):
    def __init__(self):
        super().__init__("odom_offset_node")

        # Parameters
        self.declare_parameter("odom_topic", "/r100_0599/platform/odom/filtered")
        self.declare_parameter("odom_offset_topic", "/r100_0599/platform/odom/offset")

        self.odom_topic = (
            self.get_parameter("odom_topic").get_parameter_value().string_value
        )
        self.odom_offset_topic = (
            self.get_parameter("odom_offset_topic").get_parameter_value().string_value
        )

        # Offset storage
        self.offset_pose = None

        # Publisher
        self.odom_offset_pub = self.create_publisher(
            Odometry, self.odom_offset_topic, 10
        )

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # Service
        self.trigger_srv = self.create_service(
            Trigger, "reset_odom_offset", self.handle_trigger
        )

        self.latest_odom = None
        self.get_logger().info("OdomOffsetNode initialized.")

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        # Compute offset pose if not set, default to zeros
        if self.offset_pose is None:
            self.offset_pose = copy.deepcopy(msg.pose.pose)

        # Apply offset
        offset_msg = Odometry()
        offset_msg.header = msg.header
        offset_msg.header.frame_id = "world"
        offset_msg.child_frame_id = msg.child_frame_id

        # Subtract the offset from the current pose
        offset_msg.pose.pose.position.x = (
            msg.pose.pose.position.x - self.offset_pose.position.x
        )
        offset_msg.pose.pose.position.y = (
            msg.pose.pose.position.y - self.offset_pose.position.y
        )
        offset_msg.pose.pose.position.z = (
            msg.pose.pose.position.z - self.offset_pose.position.z
        )

        # For orientation, subtract quaternion offset (not just value, need to use tf)
        q_msg = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        q_offset = [
            self.offset_pose.orientation.x,
            self.offset_pose.orientation.y,
            self.offset_pose.orientation.z,
            self.offset_pose.orientation.w,
        ]
        q_corrected = quaternion_multiply(quaternion_inverse(q_offset), q_msg)
        offset_msg.pose.pose.orientation.x = q_corrected[0]
        offset_msg.pose.pose.orientation.y = q_corrected[1]
        offset_msg.pose.pose.orientation.z = q_corrected[2]
        offset_msg.pose.pose.orientation.w = q_corrected[3]

        # Copy twist
        offset_msg.twist = copy.deepcopy(msg.twist)

        # Publish
        self.odom_offset_pub.publish(offset_msg)

    def handle_trigger(self, request, response):
        # Set the offset to the current odom pose
        if self.latest_odom is not None:
            self.offset_pose = copy.deepcopy(self.latest_odom.pose.pose)
            response.success = True
            response.message = "Offset updated to current odom pose."
            self.get_logger().info("Offset updated to current odom pose.")
        else:
            response.success = False
            response.message = "No odom message received yet."
            self.get_logger().warn("Service called before any odom message received.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OdomOffsetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
