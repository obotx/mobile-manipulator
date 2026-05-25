#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose
from action_msgs.msg import GoalStatus
import math

class MoveToObject(Node):
    def __init__(self):
        super().__init__('move_to_object_node')
        
        self.declare_parameter('obj_topic', '/nestle_choco/pose')
        self.obj_topic = self.get_parameter('obj_topic').get_parameter_value().string_value

        self.compute_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.planner_id = 'GridBased'
        self.received_pose = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Pose, 
            self.obj_topic, 
            self.pose_callback, 
            qos
        )
        self.get_logger().info(f"Listening for {self.obj_topic}...")

    def pose_callback(self, msg):
        if self.received_pose is None:
            self.received_pose = msg
            self.get_logger().info(f"Received Target: x={msg.position.x:.2f}, y={msg.position.y:.2f}")

    def run(self):
        self.get_logger().info("Starting Sequence...")

        timeout = 15.0
        start_time = self.get_clock().now()
        while rclpy.ok() and self.received_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error(f"Timeout: No pose received from {self.obj_topic}")
                return

        if self.received_pose is None:
            self.get_logger().error("Invalid pose received.")
            return

        target_x = self.received_pose.position.x
        target_y = self.received_pose.position.y
        self.get_logger().info(f"Target Acquired: ({target_x}, {target_y})")

        if not self.compute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ComputePath action server not available!')
            return
        if not self.navigate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        self.get_logger().info("Computing safe path...")
        safe_pose = self.compute_safe_stop_point(target_x, target_y)
        if safe_pose is None:
            self.get_logger().error("Failed to compute a valid path.")
            return

        final_goal = self.calculate_facing_orientation(safe_pose, target_x, target_y)
        
        self.get_logger().info("Navigating to safe position facing target...")
        self.send_navigation_goal(final_goal)

    def compute_safe_stop_point(self, tx, ty):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = tx
        goal_msg.goal.pose.position.y = ty
        goal_msg.goal.pose.orientation.w = 1.0 
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False

        future = self.compute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        status = result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED and result.path.poses:
            last_pose = result.path.poses[-1].pose
            self.get_logger().info(f"Safe Stop Point: ({last_pose.position.x:.2f}, {last_pose.position.y:.2f})")
            return last_pose
        else:
            self.get_logger().error(f"Path computation failed with code: {result.error_code}")
            return None

    def calculate_facing_orientation(self, current_pose, tx, ty):
        dx = tx - current_pose.position.x
        dy = ty - current_pose.position.y
        yaw = math.atan2(dy, dx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        self.get_logger().info(f"Calculated Facing Angle: {math.degrees(yaw):.2f} deg")
        
        final_goal = PoseStamped()
        final_goal.header.frame_id = 'map'
        final_goal.pose.position.x = current_pose.position.x
        final_goal.pose.position.y = current_pose.position.y
        final_goal.pose.position.z = 0.0
        final_goal.pose.orientation.x = 0.0
        final_goal.pose.orientation.y = 0.0
        final_goal.pose.orientation.z = qz
        final_goal.pose.orientation.w = qw
        return final_goal

    def send_navigation_goal(self, pose_stamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        goal_msg.behavior_tree = "" 

        future = self.navigate_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected!")
            return

        self.get_logger().info("Moving...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success!")
        else:
            self.get_logger().error("Navigation failed")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToObject()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()