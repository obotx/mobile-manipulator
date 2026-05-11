#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class PoseComparator(Node):
    def __init__(self):
        super().__init__('pose_comparator')
        
        self.declare_parameter('odom_topic', '/mecanum_drive_controller/odom')
        self.declare_parameter('gt_topic', '/model/MORPH_I/pose')
        self.declare_parameter('log_rate_hz', 1.0)
        
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.gt_topic = self.get_parameter('gt_topic').get_parameter_value().string_value
        self.log_rate = self.get_parameter('log_rate_hz').get_parameter_value().double_value
        self.latest_gt_pose = None
        self.last_log_time = 0.0
        
        self.create_subscription(
            Pose,
            self.gt_topic,
            self._gt_callback,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,  # Match Gazebo
                durability=DurabilityPolicy.VOLATILE
            )
        )
        self.create_subscription(
            Odometry, 
            self.odom_topic, 
            self._odom_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        )
        
        self.get_logger().info(f'Comparing: {self.odom_topic} vs {self.gt_topic}')

    def _gt_callback(self, msg: Pose):
        self.latest_gt_pose = msg

    def _odom_callback(self, msg: Odometry):
        if self.latest_gt_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time < (1.0 / self.log_rate):
            return

        # Position Error
        dx = msg.pose.pose.position.x - self.latest_gt_pose.position.x
        dy = msg.pose.pose.position.y - self.latest_gt_pose.position.y
        dz = msg.pose.pose.position.z - self.latest_gt_pose.position.z
        pos_err = math.sqrt(dx**2 + dy**2 + dz**2)

        # Yaw Error (quaternion -> yaw)
        def _quat_to_yaw(q):
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y**2 + q.z**2))

        odom_yaw = _quat_to_yaw(msg.pose.pose.orientation)
        gt_yaw   = _quat_to_yaw(self.latest_gt_pose.orientation)
        yaw_err  = math.atan2(math.sin(odom_yaw - gt_yaw), math.cos(odom_yaw - gt_yaw))

        self.get_logger().info(
            f"""
            Pos Odom:    [{msg.pose.pose.position.x:.4f}, {msg.pose.pose.position.y:.4f}, {msg.pose.pose.position.z:.4f}] m 
            Pos GT:      [{self.latest_gt_pose.position.x:.4f}, {self.latest_gt_pose.position.y:.4f}, {self.latest_gt_pose.position.z:.4f}] m 
            Pos Err:     {pos_err:.4f} m
            Yaw Odom:    {math.degrees(odom_yaw):.2f}°
            Yaw GT:      {math.degrees(gt_yaw):.2f}°
            Yaw Err:     {math.degrees(yaw_err):.2f}°
            """
        )
        self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PoseComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()