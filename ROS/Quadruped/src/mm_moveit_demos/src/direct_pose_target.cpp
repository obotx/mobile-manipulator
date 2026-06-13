#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <thread>

using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

// -------------------------------------------------------------------
// Terminal helpers for non‑blocking keyboard input
// -------------------------------------------------------------------
void setNonBlockingInput() {
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    tty.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
}

void restoreInput() {
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    tty.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &tty);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) & ~O_NONBLOCK);
}

// -------------------------------------------------------------------
// Get current end‑effector pose via TF
// -------------------------------------------------------------------
geometry_msgs::msg::Pose getCurrentPose(tf2_ros::Buffer& tf_buffer,
                                        const std::string& target_frame,
                                        const std::string& source_frame) {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("pilz_teleop"), "TF lookup failed: %s", ex.what());
        return geometry_msgs::msg::Pose();
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return pose;
}

// -------------------------------------------------------------------
// Main teleop node
// -------------------------------------------------------------------
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pilz_interruptible_teleop");
    auto logger = rclcpp::get_logger("pilz_interruptible_teleop");

    // ---------- MoveGroupInterface (for robot info and constraints) ----------
    using moveit::planning_interface::MoveGroupInterface;
    auto arm_group = MoveGroupInterface(node, "left_arm");
    const std::string PLANNING_GROUP = "left_arm";
    const std::string EE_LINK = "obotx_left_tool0";
    const std::string TARGET_FRAME = "odom_gt";

    // ---------- TF listener ----------
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ---------- Pilz MoveGroupSequence action client ----------
    auto action_client = rclcpp_action::create_client<MoveGroupSequence>(node, "/sequence_move_group");
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(logger, "MoveGroupSequence action not available. Is the Pilz pipeline loaded?");
        return 1;
    }

    // ---------- Teleop parameters ----------
    const double step_size = 0.04;           // 4 cm per key press
    const double rot_step = 0.1;             // 0.1 rad per key press
    const double planning_time = 0.5;        // seconds

    // Store the current goal handle for cancellation
    std::shared_ptr<GoalHandleMoveGroupSequence> current_goal_handle;

    RCLCPP_INFO(logger, "=== Interruptible Teleop with Pilz (MoveGroupSequence Action) ===");
    RCLCPP_INFO(logger, "Press keys: w/s (X), a/d (Y), q/e (Z), r/f (yaw), ESC to quit.");
    setNonBlockingInput();

    // Main loop
    while (rclcpp::ok()) {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) != 1) {
            rclcpp::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // ----- 1. Determine motion delta -----
        double dx = 0.0, dy = 0.0, dz = 0.0, dyaw = 0.0;
        bool valid = true;
        switch (c) {
            case 'w': dx = +step_size; break;
            case 's': dx = -step_size; break;
            case 'a': dy = +step_size; break;
            case 'd': dy = -step_size; break;
            case 'q': dz = +step_size; break;
            case 'e': dz = -step_size; break;
            case 'r': dyaw = +rot_step; break;
            case 'f': dyaw = -rot_step; break;
            case 27: restoreInput(); rclcpp::shutdown(); return 0;  // ESC
            default: valid = false; break;
        }
        if (!valid) continue;

        // ----- 2. Get current robot pose -----
        geometry_msgs::msg::Pose current_pose = getCurrentPose(*tf_buffer, TARGET_FRAME, EE_LINK);
        if (current_pose.orientation.w == 0.0 && current_pose.position.x == 0.0) {
            RCLCPP_ERROR(logger, "Failed to get current pose – skipping");
            continue;
        }

        // ----- 3. Compute target pose -----
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.x += dx;
        target_pose.position.y += dy;
        target_pose.position.z += dz;
        if (dyaw != 0.0) {
            double yaw = std::atan2(2.0 * (current_pose.orientation.w * current_pose.orientation.z +
                                          current_pose.orientation.x * current_pose.orientation.y),
                                    1.0 - 2.0 * (current_pose.orientation.y * current_pose.orientation.y +
                                                current_pose.orientation.z * current_pose.orientation.z));
            yaw += dyaw;
            target_pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), yaw));
        }

        // ----- 4. Build a Pilz sequence (single LIN command) -----
        moveit_msgs::msg::MotionSequenceRequest sequence_request;
        moveit_msgs::msg::MotionSequenceItem seq_item;
        seq_item.blend_radius = 0.0;               // Only one command, no blending
        seq_item.req.group_name = PLANNING_GROUP;
        seq_item.req.planner_id = "LIN";
        seq_item.req.allowed_planning_time = planning_time;
        seq_item.req.max_velocity_scaling_factor = 0.5;
        seq_item.req.max_acceleration_scaling_factor = 0.5;

        // Create a kinematic constraint from the target pose
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = TARGET_FRAME;
        pose_msg.pose = target_pose;
        seq_item.req.goal_constraints.push_back(
            kinematic_constraints::constructGoalConstraints(EE_LINK, pose_msg));
        sequence_request.items.push_back(seq_item);

        // ----- 5. Cancel any ongoing motion -----
        if (current_goal_handle) {
            RCLCPP_INFO(logger, "Cancelling current motion...");
            action_client->async_cancel_goal(current_goal_handle);
        }

        // ----- 6. Send the new goal -----
        auto goal_msg = MoveGroupSequence::Goal();
        goal_msg.request = sequence_request;
        goal_msg.planning_options.planning_scene_diff.is_diff = true;
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;

        auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
        send_goal_options.goal_response_callback = [&logger](auto goal_handle) {
            if (!goal_handle) RCLCPP_ERROR(logger, "Goal rejected");
            else RCLCPP_INFO(logger, "Goal accepted");
        };
        send_goal_options.result_callback = [&logger](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(logger, "Motion finished");
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_INFO(logger, "Motion cancelled");
            } else {
                // Optional: print error code from result.result->response.error_code.val
                RCLCPP_WARN(logger, "Motion finished with result code: %d", result.code);
            }
        };

        auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future, std::chrono::milliseconds(500)) !=
            rclcpp::FutureReturnCode::SUCCESS || !goal_handle_future.get()) {
            RCLCPP_ERROR(logger, "Failed to send goal");
            continue;
        }

        current_goal_handle = goal_handle_future.get();
        RCLCPP_INFO(logger, "Moving to (%.3f, %.3f, %.3f)", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        rclcpp::sleep_for(std::chrono::milliseconds(50)); // debounce
    }

    restoreInput();
    rclcpp::shutdown();
    return 0;
}