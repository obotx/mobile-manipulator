#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <thread>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);

    auto node = std::make_shared<rclcpp::Node>(
        "auto_arm_selector",
        options);

    auto logger = node->get_logger();

    node->declare_parameter("arm_side", "auto");
    node->declare_parameter("target_x",2.5);
    node->declare_parameter("target_y",1.25);
    node->declare_parameter("target_z",1.2);

    std::string arm_side = node->get_parameter("arm_side").as_string();

    double target_x = node->get_parameter("target_x").as_double();
    double target_y = node->get_parameter("target_y").as_double();
    double target_z = node->get_parameter("target_z").as_double();

    const std::string target_frame = "odom_gt";

    RCLCPP_INFO( logger, "Input arm_side = %s", arm_side.c_str());

    RCLCPP_INFO(
        logger,
        "Target [%s] = [%.3f %.3f %.3f]",
        target_frame.c_str(),
        target_x,
        target_y,
        target_z);

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // MOVEIT
    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface temp_group(node, "left_arm");

    moveit::core::RobotStatePtr current_state = temp_group.getCurrentState(5.0);

    if (!current_state)
    {
        RCLCPP_ERROR(
            logger,
            "Failed to get current robot state");

        rclcpp::shutdown();
        spinner.join();

        return 1;
    }

    current_state->update();

    std::string model_frame =
        current_state->getRobotModel()->getModelFrame();

    RCLCPP_INFO(
        logger,
        "MoveIt model frame = %s",
        model_frame.c_str());

    // END EFFECTORS
    const std::string left_tip =
        "obotx_left_tool0";

    const std::string right_tip =
        "obotx_right_tool0";

    Eigen::Isometry3d left_tf = current_state->getGlobalLinkTransform(left_tip);
    Eigen::Vector3d left_ee = left_tf.translation();

    geometry_msgs::msg::PoseStamped left_pose_model;
    left_pose_model.header.frame_id = model_frame;
    left_pose_model.header.stamp = rclcpp::Time(0);
    left_pose_model.pose.position.x = left_ee.x();
    left_pose_model.pose.position.y = left_ee.y();
    left_pose_model.pose.position.z = left_ee.z();
    left_pose_model.pose.orientation.w = 1.0;

    Eigen::Isometry3d right_tf = current_state->getGlobalLinkTransform(right_tip);
    Eigen::Vector3d right_ee = right_tf.translation();
    geometry_msgs::msg::PoseStamped right_pose_model;
    right_pose_model.header.frame_id = model_frame;
    right_pose_model.header.stamp = rclcpp::Time(0);
    right_pose_model.pose.position.x = right_ee.x();
    right_pose_model.pose.position.y = right_ee.y();
    right_pose_model.pose.position.z = right_ee.z();
    right_pose_model.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped left_pose_odom;
    geometry_msgs::msg::PoseStamped right_pose_odom;

    try
    {
        left_pose_odom =
            tf_buffer.transform(
                left_pose_model,
                target_frame,
                tf2::durationFromSec(1.0));

        right_pose_odom =
            tf_buffer.transform(
                right_pose_model,
                target_frame,
                tf2::durationFromSec(1.0));
    }
    catch (tf2::TransformException & ex)
    {
        RCLCPP_ERROR(
            logger,
            "TF transform failed: %s",
            ex.what());

        rclcpp::shutdown();
        spinner.join();

        return 1;
    }

    Eigen::Vector3d left_odom(
        left_pose_odom.pose.position.x,
        left_pose_odom.pose.position.y,
        left_pose_odom.pose.position.z);

    Eigen::Vector3d right_odom(
        right_pose_odom.pose.position.x,
        right_pose_odom.pose.position.y,
        right_pose_odom.pose.position.z);
    Eigen::Vector3d target(
        target_x,
        target_y,
        target_z);

    double dist_left = (left_odom - target).norm();
    double dist_right = (right_odom - target).norm();

    RCLCPP_INFO(
        logger,
        "LEFT EE odom_gt  = [%.3f %.3f %.3f]",
        left_odom.x(),
        left_odom.y(),
        left_odom.z());

    RCLCPP_INFO(
        logger,
        "RIGHT EE odom_gt = [%.3f %.3f %.3f]",
        right_odom.x(),
        right_odom.y(),
        right_odom.z());

    RCLCPP_INFO(
        logger,
        "Distance LEFT  = %.3f",
        dist_left);

    RCLCPP_INFO(
        logger,
        "Distance RIGHT = %.3f",
        dist_right);

    // ARM SELECTION
    std::string selected_arm;

    if (arm_side == "left")
    {
        selected_arm = "left";
        RCLCPP_INFO(
            logger,
            "Forced LEFT arm");
    }
    else if (arm_side == "right")
    {
        selected_arm = "right";
        RCLCPP_INFO(
            logger,
            "Forced RIGHT arm");
    }
    else
    {
        selected_arm = (dist_left < dist_right) ? "left" : "right";
        RCLCPP_INFO(
            logger,
            "AUTO arm selection");
    }

    std::string planning_group = selected_arm + "_arm";
    std::string tip_link = (selected_arm == "left") ? left_tip : right_tip;

    RCLCPP_INFO(
        logger,
        "Selected arm = %s",
        selected_arm.c_str());

    // MOVE GROUP
    MoveGroupInterface arm_group(node, planning_group);
    arm_group.setPoseReferenceFrame(target_frame);
    arm_group.setEndEffectorLink(tip_link);
    arm_group.setPlannerId("RRTConnect");
    arm_group.setPlanningTime(10.0);
    arm_group.allowReplanning(true);
    arm_group.setNumPlanningAttempts(20);
    arm_group.setGoalTolerance(0.01);
    arm_group.setMaxVelocityScalingFactor(0.3);
    arm_group.setMaxAccelerationScalingFactor(0.3);

    arm_group.setStartStateToCurrentState();
    moveit::core::RobotState start_state(*arm_group.getCurrentState());
    start_state.update();
    start_state.enforceBounds();  
    bool valid =  start_state.satisfiesBounds();  
    RCLCPP_INFO(
        logger,
        "Start state valid after enforceBounds = %s",
        valid ? "YES" : "NO");  
    arm_group.setStartState(start_state);

    Eigen::Isometry3d current_ee_tf =
        current_state->getGlobalLinkTransform(
            tip_link);

    Eigen::Quaterniond q(
        current_ee_tf.rotation());
    geometry_msgs::msg::PoseStamped target_pose;

    target_pose.header.frame_id = target_frame;
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = target_x;
    target_pose.pose.position.y = target_y;
    target_pose.pose.position.z = target_z;
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();
    arm_group.clearPoseTargets();

    arm_group.setPoseTarget(
        target_pose,
        tip_link);


    // PLAN
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    RCLCPP_INFO(
        logger,
        "Planning trajectory...");

    auto result =
        arm_group.plan(plan);

    bool success =
        (result ==
        moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(
            logger,
            "Planning SUCCESS");

        auto exec_result =
            arm_group.execute(plan);

        if (exec_result ==
            moveit::core::MoveItErrorCode::SUCCESS)
        {
        RCLCPP_INFO(
            logger,
            "Execution SUCCESS");
        }
        else
        {
        RCLCPP_ERROR(
            logger,
            "Execution FAILED");
        }
    }
    else
    {
        RCLCPP_ERROR(
            logger,
            "Planning FAILED");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}