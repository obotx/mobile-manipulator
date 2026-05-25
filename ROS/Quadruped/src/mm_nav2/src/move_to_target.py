#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class FaceTargetNode(Node):
    def __init__(self):
        super().__init__('face_target_node')
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_rx', 0.0)
        self.declare_parameter('target_ry', 0.0)
        self.declare_parameter('target_rz', 0.0)

        self.declare_parameter('face_to_target', True)

        self.tx = self.get_parameter('target_x').value
        self.ty = self.get_parameter('target_y').value
        self.rx = self.get_parameter('target_rx').value
        self.ry = self.get_parameter('target_ry').value
        self.rz = self.get_parameter('target_rz').value
        self.face_to_target = self.get_parameter('face_to_target').value

        self.compute_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.planner_id = 'GridBased'

    def euler_to_quat(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def run(self):
        if not self.compute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ComputePathToPose unavailable')
            return

        if not self.navigate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose unavailable')
            return

        safe_pose = self.compute_safe_stop(
            self.tx,
            self.ty
        )

        if safe_pose is None:
            self.get_logger().error('Failed path compute')
            return

        if self.face_to_target:
            goal = self.make_face_goal(safe_pose, self.tx, self.ty)
        else:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = safe_pose.position.x
            goal.pose.position.y = safe_pose.position.y
            goal.pose.position.z = 0.0
            qx, qy, qz, qw = (
                self.euler_to_quat(
                    self.rx,
                    self.ry,
                    self.rz
                )
            )

            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        self.navigate(goal)

    def compute_safe_stop(self, tx, ty):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = tx
        goal.goal.pose.position.y = ty
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = self.planner_id
        future = self.compute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            future
        )

        handle = future.result()
        if not handle or not handle.accepted:
            return None
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = (result_future.result())
        if (result.status == GoalStatus.STATUS_SUCCEEDED):
            return result.result.path.poses[-1].pose
        return None

    def make_face_goal(self, pose, tx, ty):
        dx = tx - pose.position.x
        dy = ty - pose.position.y

        yaw = math.atan2(dy, dx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = (pose.position.x)
        goal.pose.position.y = (pose.position.y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        return goal

    def navigate(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            future
        )

        handle = future.result()
        if (not handle or not handle.accepted):
            self.get_logger().error('Navigation rejected')
            return

        result_future = (handle.get_result_async())
        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        status = result_future.result().status
        if (status == GoalStatus.STATUS_SUCCEEDED):
            self.get_logger().info('Navigation SUCCESS')
        else:
            self.get_logger().error('Navigation FAILED')


def main(args=None):
    rclpy.init(args=args)
    node = FaceTargetNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()