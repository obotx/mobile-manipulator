#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster

class GzPoseToTf(Node):
    def __init__(self):
        super().__init__('gz_pose_to_tf')

        # Declare only your custom parameters
        self.declare_parameter('pose_topic', '/MORPH_I/pose')
        self.declare_parameter('parent_frame', 'odom_gt')
        self.declare_parameter('child_frame', 'obotx_base_footprint_platform')
        
        # Retrieve values
        self.pose_topic = self.get_parameter('pose_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Pose, self.pose_topic, self.pose_callback, 10)
        
        self.get_logger().info(
            f'Bridge active: {self.pose_topic} '
            f'({self.parent_frame} → {self.child_frame})'
        )

    def pose_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        # geometry_msgs/Pose contains position & orientation directly
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        t.transform.rotation = msg.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GzPoseToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()