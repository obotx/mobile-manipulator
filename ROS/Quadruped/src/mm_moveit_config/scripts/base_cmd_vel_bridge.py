#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import GoalResponse
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class BasePositionControlBridge(Node):
    def __init__(self):
        super().__init__('base_cmd_vel_bridge')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.declare_parameter('cmd_vel_topic', '/mecanum_drive_controller/cmd_vel')
        self.declare_parameter('odom_topic', '/mecanum_drive_controller/odom')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('Kp_xy', 10.0) 
        self.declare_parameter('Kp_theta', 10.0)
        self.declare_parameter('verbose', True)
        self.declare_parameter('control_rate', 50.0)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.get_parameter('cmd_vel_topic').value, 100)
        self.frame_id = self.get_parameter('frame_id').value
        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.Kp_xy = self.get_parameter('Kp_xy').value
        self.Kp_theta = self.get_parameter('Kp_theta').value
        self.verbose = self.get_parameter('verbose').value
        self.control_rate = self.get_parameter('control_rate').value

        self.current_pose = [0.0, 0.0, 0.0]
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 100)

        self._action_server = ActionServer(
            self, FollowJointTrajectory, '/base_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,  
            cancel_callback=self.cancel_callback
        )
        self.get_logger().debug('BasePositionControlBridge ready')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().error('Cancel requested for base trajectory')
        return True

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

    def execute_callback(self, goal_handle):
        self.get_logger().debug('Executing base trajectory')
        trajectory = goal_handle.request.trajectory
        traj_duration = trajectory.points[-1].time_from_start.sec + trajectory.points[-1].time_from_start.nanosec * 1e-9
                
        try:
            idx_x = trajectory.joint_names.index('position/x')
            idx_y = trajectory.joint_names.index('position/y')
            idx_theta = trajectory.joint_names.index('position/theta')
        except ValueError:
            self.get_logger().error("Missing base DOFs in trajectory.")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        start_wall = time.monotonic()
        last_log = start_wall
        rate = self.create_rate(self.control_rate)
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().error('Trajectory cancelled')
                goal_handle.canceled()
                self._publish_stop()
                return FollowJointTrajectory.Result()

            elapsed = time.monotonic() - start_wall
            if elapsed >= traj_duration:
                break

            idx = 0
            while idx < len(trajectory.points) - 1:
                t_next = trajectory.points[idx+1].time_from_start.sec + trajectory.points[idx+1].time_from_start.nanosec * 1e-9
                if elapsed < t_next: break
                idx += 1

            pt = trajectory.points[idx]
            x_t, y_t, th_t = pt.positions[idx_x], pt.positions[idx_y], pt.positions[idx_theta]
            x_c, y_c, th_c = self.current_pose

            dx = x_t - x_c
            dy = y_t - y_c
            dth = math.atan2(math.sin(th_t - th_c), math.cos(th_t - th_c))

            vx = self.Kp_xy * dx
            vy = self.Kp_xy * dy
            wz = self.Kp_theta * dth

            vx = float(max(-self.max_lin, min(self.max_lin, vx)))
            vy = float(max(-self.max_lin, min(self.max_lin, vy)))
            wz = float(max(-self.max_ang, min(self.max_ang, wz)))

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.angular.z = wz
            self.cmd_vel_pub.publish(msg)

            now = time.monotonic()
            if now - last_log >= 0.5 and self.verbose:
                err = math.sqrt(dx**2 + dy**2)
                self.get_logger().info(f"t={elapsed:.1f}s | Err={err:.3f}m | Odom=[{x_c:.2f}, {y_c:.2f}]")
                last_log = now

            rate.sleep() 

        self._publish_stop()
        self.get_logger().debug('Base trajectory completed')
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def _publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BasePositionControlBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()