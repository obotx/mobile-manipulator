#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <memory>
#include <thread>
#include <string>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true);

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_state_controller",
        options);

    auto logger = node->get_logger();

    node->declare_parameter(
        "group_name",
        "gripper_left");

    node->declare_parameter(
        "state_name",
        "open");

    std::string group_name =
        node->get_parameter("group_name")
            .as_string();

    std::string state_name =
        node->get_parameter("state_name")
            .as_string();

    RCLCPP_INFO(
        logger,
        "Group      : %s",
        group_name.c_str());

    RCLCPP_INFO(
        logger,
        "State      : %s",
        state_name.c_str());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() {
        executor.spin();
    });

    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface move_group(
        node,
        group_name);

    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(10);
    move_group.allowReplanning(true);
    move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(0.3);
        move_group.setMaxAccelerationScalingFactor(0.3);

    bool target_ok =
        move_group.setNamedTarget(
            state_name);

    if (!target_ok)
    {
        RCLCPP_ERROR(
            logger,
            "Named state '%s' does not exist for group '%s'",
            state_name.c_str(),
            group_name.c_str());

        rclcpp::shutdown();
        spinner.join();

        return 1;
    }

    // PLAN
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(
        logger,
        "Planning...");

    auto result = move_group.plan(plan);
    bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_ERROR(
            logger,
            "Planning FAILED");

        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    RCLCPP_INFO(
        logger,
        "Planning SUCCESS");

    // EXECUTE
    auto exec_result = move_group.execute(plan);
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

    rclcpp::shutdown();
    spinner.join();
    return 0;
}