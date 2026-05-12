#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <mm_moveit_demos/pick_place_task.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

moveit_msgs::msg::CollisionObject createTable(
    const std::string &table_name, const geometry_msgs::msg::Pose &pose) {
  moveit_msgs::msg::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = "odom";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<
          shape_msgs::msg::SolidPrimitive::BOX>());
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_X) =
      0.2;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Y) =
      0.2;
  object.primitives[0].dimensions.at(shape_msgs::msg::SolidPrimitive::BOX_Z) =
      0.25;
  object.primitive_poses.push_back(pose);
  object.primitive_poses.back().position.z +=
      0.5 *
      object.primitives[0].dimensions.at(
          shape_msgs::msg::SolidPrimitive::BOX_Z);  
  return object;
}

moveit_msgs::msg::CollisionObject createObject(
    const geometry_msgs::msg::Pose &pose) {
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "odom";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<
          shape_msgs::msg::SolidPrimitive::CYLINDER>());
  object.primitives[0].dimensions.at(
      shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT) = 0.3;
  object.primitives[0].dimensions.at(
      shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS) = 0.02;
  object.primitive_poses.push_back(pose);
  object.primitive_poses.back().position.z +=
      0.5 * object.primitives[0].dimensions.at(
                shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT);
  return object;
}

std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
loadPlanningSceneMonitor(const rclcpp::Node::SharedPtr &node) {
  auto robot_model_loader =
      std::make_shared<robot_model_loader::RobotModelLoader>(
          node, "robot_description");
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          node, robot_model_loader);

  if (planning_scene_monitor->getPlanningScene()) {
    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->requestPlanningSceneState();
    // Wait for complete state to be received
    planning_scene_monitor->getStateMonitor()->waitForCurrentState(node->now());
    planning_scene_monitor->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
        planning_scene_monitor::PlanningSceneMonitor::
            DEFAULT_PLANNING_SCENE_TOPIC);
  }
  return planning_scene_monitor;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_place_demo", "", options);
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  auto psm = loadPlanningSceneMonitor(node);
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(psm);
    scene->processCollisionObjectMsg(createTable(
        "table_source",
        tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, 1, 0)))));
    scene->processCollisionObjectMsg(createTable(
        "table_target",
        tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, -1, 0)))));
    scene->processCollisionObjectMsg(createObject(
        tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, 1, 0.25)))));
  }  // Unlock PlanningScene
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm);
    moveit::core::RobotState current_state = scene->getCurrentState();
    
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    
    // Check for any collision in the current state
    scene->checkCollision(req, res, current_state);
    
    if (res.collision) {
      RCLCPP_WARN(LOGGER, "⚠️ COLLISION DETECTED AT STARTUP!");
      for (const auto& contact : res.contacts) {
        RCLCPP_WARN(LOGGER, "  ↔ %s  -  %s", 
                    contact.first.first.c_str(), contact.first.second.c_str());
      }
    } else {
      RCLCPP_INFO(LOGGER, "✅ No collisions detected at startup.");
    }
  }
  
  psm->triggerSceneUpdateEvent(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  rclcpp::sleep_for(500ms);

  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm);

    // Copy ACM
    collision_detection::AllowedCollisionMatrix acm =
        scene->getAllowedCollisionMatrix();

    // Base and wheel links
    std::vector<std::string> base_links = {
        "obotx_base_link_platform",
        "obotx_front_left_wheel",
        "obotx_front_right_wheel",
        "obotx_back_left_wheel",
        "obotx_back_right_wheel"
    };

    // Environment objects
    std::vector<std::string> environment_objects = {
        "table_source",
        "table_target",
    };

    // Allow collisions
    for (const auto& base_link : base_links)
    {
      for (const auto& env : environment_objects)
      {
        acm.setEntry(base_link, env, true);
      }
    }

    // Apply ACM back
    scene->getAllowedCollisionMatrixNonConst() = acm;

    // Notify planning scene update
    psm->triggerSceneUpdateEvent(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    RCLCPP_INFO(LOGGER,
                "Allowed base/wheel collisions with tables and object");
  }

  PickPlaceTask::Parameters pick_place_parameters;
  //  pick_place_parameters.arm_group_name = "stretch_arm";
  pick_place_parameters.hand_group_name = "gripper_left";
  pick_place_parameters.end_effector_name = "ee_left_gripper";
  pick_place_parameters.hand_frame = "obotx_left_tool0";
  pick_place_parameters.object_name = "object";
  pick_place_parameters.hand_open_pose = "open";
  pick_place_parameters.hand_close_pose = "close";
  pick_place_parameters.mobile_base_arm_group_name = "left_arm_with_base";
  pick_place_parameters.place_pose.header.frame_id = "odom";
  pick_place_parameters.place_pose.pose =
      tf2::toMsg(Eigen::Isometry3d(Eigen::Translation3d(1, -1, 0.4)));

  PickPlaceTask pick_place_task(node, pick_place_parameters);
  if (!pick_place_task.plan()) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to plan");
  }

  // Keep introspection alive
  spinning_thread.join();
  rclcpp::shutdown();
  return 0;
}