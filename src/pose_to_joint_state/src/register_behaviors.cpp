#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/json_serialization.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <pose_to_joint_state/pose_to_joint_state.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace pose_to_joint_state
{
class PoseToJointStateBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<PoseToJointState>(factory, "PoseToJointState", shared_resources);
    register_ros_msg<trajectory_msgs::msg::JointTrajectory>();
  }
};
}  // namespace pose_to_joint_state

PLUGINLIB_EXPORT_CLASS(pose_to_joint_state::PoseToJointStateBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
