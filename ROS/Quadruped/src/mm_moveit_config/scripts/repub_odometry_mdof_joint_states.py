#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Transform, Twist

class OdometryToMultiDOF(Node):
    def __init__(self):
        super().__init__('odometry_mdof_republisher')
        self.declare_parameter('odom_topic', '/mecanum_drive_controller/odom')
        self.declare_parameter('mdof_topic', '/multi_dof_joint_states')
        self.declare_parameter('joint_name', 'position')
        
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)
        self.mdof_pub = self.create_publisher(
            MultiDOFJointState, self.get_parameter('mdof_topic').value, 10)
        
        self.get_logger().info('Odometry republisher ready')

    def odom_callback(self, msg: Odometry):
        mdof = MultiDOFJointState()
        mdof.header = msg.header
        mdof.joint_names = [self.get_parameter('joint_name').value]
        
        transform = Transform()
        transform.translation.x = msg.pose.pose.position.x
        transform.translation.y = msg.pose.pose.position.y
        transform.translation.z = msg.pose.pose.position.z
        transform.rotation = msg.pose.pose.orientation
        mdof.transforms = [transform]
        
        twist = Twist()
        twist.linear = msg.twist.twist.linear
        twist.angular = msg.twist.twist.angular
        mdof.twist = [twist]
        
        self.mdof_pub.publish(mdof)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToMultiDOF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()