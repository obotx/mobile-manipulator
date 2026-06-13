#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class WorkspaceRegionVisualizer(Node):
    def __init__(self):
        super().__init__('workspace_region_visualizer')
        self.marker_pub = self.create_publisher(Marker, '/workspace_region_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_region_marker)
        self.get_logger().info("Workspace Region Visualizer started")
        self.get_logger().info("Add Marker display in RViz: /workspace_region_marker")

    def publish_region_marker(self):
        marker = Marker()
        marker.header.frame_id = "obotx_base_footprint_platform"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace_region"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.8
        marker.color.a = 0.3
        
        radius = 2.0
        z_height = 0.05
        
        points = []
        p0 = Point(x=0.0, y=0.0, z=z_height)
        
        angle_start = math.radians(45)
        angle_end = math.radians(-45)
        num_segments = 60               # smoother arc (was 30)
        angle_range = angle_end - angle_start
        
        for i in range(num_segments):
            angle1 = angle_start + (i / num_segments) * angle_range
            angle2 = angle_start + ((i + 1) / num_segments) * angle_range
            
            p1 = Point(x=radius * math.cos(angle1), 
                      y=radius * math.sin(angle1), 
                      z=z_height)
            p2 = Point(x=radius * math.cos(angle2), 
                      y=radius * math.sin(angle2), 
                      z=z_height)
            
            points.extend([p0, p1, p2])
            
        marker.points = points
        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = WorkspaceRegionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()