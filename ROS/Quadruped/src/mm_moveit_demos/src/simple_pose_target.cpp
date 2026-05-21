#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  // Start ROS 2
  rclcpp::init(argc, argv);

  // Node options
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto node = std::make_shared<rclcpp::Node>(
      "moveit_pose_target_example",
      options);

  auto const logger = rclcpp::get_logger("plan_around_objects");

  // Declare parameters
  node->declare_parameter("target_x", 1.0);
  node->declare_parameter("target_y", 1.6);
  node->declare_parameter("target_z", 0.8);

  // Get parameters
  double target_x = node->get_parameter_or("target_x", 1.0);

  double target_y = node->get_parameter_or("target_y", 1.6);

  double target_z = node->get_parameter_or("target_z", 0.8);

  RCLCPP_INFO(logger, "Target Position: x=%.2f y=%.2f z=%.2f",
              target_x, target_y, target_z);

  // Executor thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto spinner = std::thread([&executor]() {
    executor.spin();
  });

  using moveit::planning_interface::MoveGroupInterface;

  // Move group
  auto arm_group_interface = MoveGroupInterface(node, "left_arm_with_base");

  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnect");
  arm_group_interface.setPlanningTime(10.0);

  arm_group_interface.setGoalPositionTolerance(0.01);
  arm_group_interface.setGoalOrientationTolerance(0.01);

  arm_group_interface.setPoseReferenceFrame("world");

  // Logging
  RCLCPP_INFO(logger, "Planning pipeline: %s",
              arm_group_interface.getPlanningPipelineId().c_str());

  RCLCPP_INFO(logger, "Planner ID: %s",
              arm_group_interface.getPlannerId().c_str());

  // Visual tools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{
          node,
          "world",
          rviz_visual_tools::RVIZ_MARKER_TOPIC,
          arm_group_interface.getRobotModel()};

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const draw_title = [&moveit_visual_tools](auto text)
  {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();

    moveit_visual_tools.publishText(
        text_pose,
        text,
        rviz_visual_tools::WHITE,
        rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&moveit_visual_tools](auto text)
  {
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = arm_group_interface.getRobotModel()
                 ->getJointModelGroup("left_arm_with_base")]
      (auto const trajectory)
  {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  // Target pose
  geometry_msgs::msg::Pose target_pose;

  target_pose.orientation.w = 1.0;

  target_pose.position.x = target_x;
  target_pose.position.y = target_y;
  target_pose.position.z = target_z;

  arm_group_interface.clearPoseTargets();
  arm_group_interface.clearPathConstraints();
  arm_group_interface.stop();

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.trigger();

  // arm_group_interface.setJointValueTarget(
  //     target_pose,
  //     "obotx_left_tool0");


  // Get robot state
  moveit::core::RobotStatePtr current_state =
      arm_group_interface.getCurrentState(10.0);

  const moveit::core::JointModelGroup* joint_model_group =
      current_state->getJointModelGroup("left_arm_with_base");

  // Compute IK
  bool found_ik = current_state->setFromIK(
      joint_model_group,
      target_pose,
      "obotx_left_tool0",
      0.1);   // timeout

  if (found_ik)
  {
    RCLCPP_INFO(logger, "IK solution found");

    std::vector<double> joint_values;

    current_state->copyJointGroupPositions(
        joint_model_group,
        joint_values);

    // Set joint target from IK result
    arm_group_interface.setJointValueTarget(joint_values);
  }
  else
  {
    RCLCPP_ERROR(logger, "IK solution NOT found");
  }

  // // Collision object
  // auto const collision_object =
  //     [frame_id = arm_group_interface.getPlanningFrame(),
  //      &node,
  //      &logger]
  // {
  //   moveit_msgs::msg::CollisionObject collision_object;

  //   collision_object.header.frame_id = frame_id;
  //   collision_object.header.stamp = node->now();

  //   collision_object.id = "box1";

  //   shape_msgs::msg::SolidPrimitive primitive;

  //   primitive.type = primitive.BOX;
  //   primitive.dimensions.resize(3);

  //   primitive.dimensions[primitive.BOX_X] = 0.2;
  //   primitive.dimensions[primitive.BOX_Y] = 1.0;
  //   primitive.dimensions[primitive.BOX_Z] = 0.05;

  //   geometry_msgs::msg::Pose box_pose;

  //   box_pose.position.x = 0.8;
  //   box_pose.position.y = 0.0;
  //   box_pose.position.z = 0.3;

  //   box_pose.orientation.w = 1.0;

  //   collision_object.primitives.push_back(primitive);
  //   collision_object.primitive_poses.push_back(box_pose);

  //   collision_object.operation = collision_object.ADD;

  //   return collision_object;
  // }();

  // // Planning scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // planning_scene_interface.applyCollisionObject(collision_object);

  // Prompt
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");

  draw_title("Planning");

  moveit_visual_tools.trigger();

  // Plan
  auto const [success, plan] = [&arm_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;

    auto const ok =
        static_cast<bool>(arm_group_interface.plan(msg));

    return std::make_pair(ok, msg);
  }();

  // Execute
  if (success)
  {
    draw_trajectory_tool_path(plan.trajectory);

    moveit_visual_tools.trigger();

    // arm_group_interface.execute(plan);

    RCLCPP_INFO(logger, "Planning successfully");
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();

    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  spinner.join();

  return 0;
}