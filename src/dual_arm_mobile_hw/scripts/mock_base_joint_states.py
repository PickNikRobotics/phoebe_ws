import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MockJointStatePublisher(Node):
    def __init__(self):
        super().__init__('mock_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_zero_joint_states)  # 10 Hz
        self.get_logger().info("Publishing zero joint states at 10 Hz")

    def publish_zero_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define all joints including the new ones
        msg.name = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
            # "linear_x_joint",
            # "linear_y_joint",
            # "rotational_yaw_joint"
        ]

        # Set positions, velocities, and efforts to zero
        num_joints = len(msg.name)
        msg.position = [0.0] * num_joints
        msg.velocity = [0.0] * num_joints
        msg.effort = [0.0] * num_joints

        self.publisher_.publish(msg)
        self.get_logger().debug("Published zero joint states")

def main(args=None):
    rclpy.init(args=args)
    node = MockJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

