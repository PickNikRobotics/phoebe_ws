#include "pose_to_joint_state/pose_to_joint_state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spdlog/spdlog.h"
#include "tf2/utils.hpp"

#include "moveit_studio_agent_msgs/msg/robot_joint_state.hpp"

namespace pose_to_joint_state
{
PoseToJointState::PoseToJointState(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList PoseToJointState::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>(
               kPortIDDesiredPose, "{pose}", "The desired pose of the frame represented in the world frame."),
           BT::OutputPort<moveit_studio_agent_msgs::msg::RobotJointState>(
               kPortIDTargetJointState, "{joint_state}",
               "The target joint state of the frame, assuming the joints are currently at zero.") };
}

BT::KeyValueVector PoseToJointState::metadata()
{
  return { { "description", "Take in the pose of the base in the tag frame and the desired pose of the base in the tag "
                            "frame and compute a target joint state for a point to point plan" },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus PoseToJointState::tick()
{
  const auto ports =
      moveit_studio::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDDesiredPose));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required inputs: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [desired_pose] = ports.value();

  moveit_studio_agent_msgs::msg::RobotJointState target_joint_state = moveit_studio_agent_msgs::msg::RobotJointState();
  target_joint_state.joint_state.name = { "linear_x_joint", "linear_y_joint", "rotational_yaw_joint" };
  target_joint_state.joint_state.position = { desired_pose.pose.position.x, desired_pose.pose.position.y,
                                              tf2::getYaw(desired_pose.pose.orientation) };
  setOutput<moveit_studio_agent_msgs::msg::RobotJointState>(kPortIDTargetJointState, target_joint_state);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pose_to_joint_state
