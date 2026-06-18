import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import argparse

class StaticTFPublisher(Node):
    def __init__(self, parent_frame, offset_x, offset_y, offset_z):
        super().__init__('landmark_static_tf_publisher')
        
        # 1. Create the static broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # 2. Create the transform message
        self.transform = TransformStamped()
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = parent_frame
        self.transform.child_frame_id = 'landmark'
        
        # 3. Set translation
        self.transform.transform.translation.x = float(offset_x)
        self.transform.transform.translation.y = float(offset_y)
        self.transform.transform.translation.z = float(offset_z)
        
        # 4. Set rotation (Yaw = 3.14159265, Roll = 0, Pitch = 0)
        # A 180-degree rotation around Z is quaternion [x=0, y=0, z=1, w=0]
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 1.0
        self.transform.transform.rotation.w = 0.0
        
        # 5. Publish it once (static transforms only need to be sent once)
        self.tf_broadcaster.sendTransform(self_transform=self.transform)
        self.get_logger().info(
            f"Published static transform: '{parent_frame}' -> 'landmark' "
            f"(x={offset_x}, y={offset_y}, z={offset_z}, yaw=3.14159)"
        )

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--parent-frame', type=str, default='base_link')
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = StaticTFPublisher(args.parent_frame, args.x, args.y, args.z)
    
    try:
        # Keep the node alive so the transform stays in the tf2 tree
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()