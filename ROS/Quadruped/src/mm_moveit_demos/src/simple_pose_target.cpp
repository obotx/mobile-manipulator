#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>
#include <chrono>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
 
  auto const node = std::make_shared<rclcpp::Node>("simple_pose_target");
  auto const logger = rclcpp::get_logger("simple_pose_target");
 
  // 🕒 STEP 1: Wait for /joint_states to publish NON-ZERO timestamp
  // This bypasses MoveIt's state monitor rejecting time=0 messages
  RCLCPP_INFO(logger, "Waiting for /joint_states with valid timestamp...");
  bool has_valid_timestamp = false;
  auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&has_valid_timestamp](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->header.stamp.sec > 0 || msg->header.stamp.nanosec > 0) {
        has_valid_timestamp = true;
      }
    });
 
  auto start_wait = node->now();
  while (rclcpp::ok() && !has_valid_timestamp && 
         (node->now() - start_wait).seconds() < 10.0) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
 
  if (!has_valid_timestamp) {
    RCLCPP_ERROR(logger, "❌ Timed out waiting for valid /joint_states timestamp");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "✅ /joint_states has valid timestamp. Initializing MoveIt...");
 
  // 🤖 STEP 2: Now create MoveGroupInterface (state monitor will sync correctly)
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "left_arm_with_base");
 
  // Optional: Let state monitor fully sync
  std::this_thread::sleep_for(std::chrono::seconds(1));
 
  // Configuration
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnect");
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPoseReferenceFrame("world");
  move_group.setWorkspace(-20, -20, -20, 20, 20, 20);
 
  // Target setup
  const std::string eef_link = "obotx_left_tool0";
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = node->now();
    msg.pose.position.x = 0.6416361927986145;
    msg.pose.position.y = 1.0487403869628906;
    msg.pose.position.z = 0.9514437913894653;
    msg.pose.orientation.x = -0.03054502047598362;
    msg.pose.orientation.y = 0.544776201248169;
    msg.pose.orientation.z = 0.046780213713645935;
    msg.pose.orientation.w = 0.8367182612419128;
    return msg;
  }();
 
  // IK & Planning
  RCLCPP_INFO(logger, "Solving IK...");
  if (!move_group.setJointValueTarget(arm_target_pose.pose, eef_link)) {
    RCLCPP_ERROR(logger, "❌ IK failed");
    rclcpp::shutdown();
    return 1;
  }
 
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan)) {
    RCLCPP_INFO(logger, "✅ Planning succeeded!");
    if (!plan.trajectory.joint_trajectory.joint_names.empty()) {
      RCLCPP_INFO(logger, "   Trajectory points: %zu", 
                  plan.trajectory.joint_trajectory.points.size());
    }
  } else {
    RCLCPP_ERROR(logger, "❌ Planning failed!");
  }
 
  rclcpp::shutdown();
  return 0;
}