#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class MapPadder(Node):
    def __init__(self):
        super().__init__('map_padder')
        
        self.declare_parameter('padding_x', 1.0)
        self.declare_parameter('padding_y', 1.0)
        self.padding_x = self.get_parameter('padding_x').value
        self.padding_y = self.get_parameter('padding_y').value
        
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(
            OccupancyGrid,
            'padded_map',
            qos
        )        

        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos
        )
        
        self.last_map = None
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f"Map Padder Started")

    def listener_callback(self, msg):
        self.last_map = msg
        self.process_and_publish(msg)

    def timer_callback(self):
        if self.last_map is not None:
            self.process_and_publish(self.last_map)

    def process_and_publish(self, msg):
        res = msg.info.resolution
        
        pad_px_x = int(math.ceil(self.padding_x / res))
        pad_px_y = int(math.ceil(self.padding_y / res))
        
        new_msg = OccupancyGrid()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = msg.header.frame_id
        
        new_width = msg.info.width + 2 * pad_px_x
        new_height = msg.info.height + 2 * pad_px_y
        
        new_msg.info.map_load_time = msg.info.map_load_time
        new_msg.info.resolution = res
        new_msg.info.width = new_width
        new_msg.info.height = new_height
        
        new_msg.info.origin.position.x = msg.info.origin.position.x - (pad_px_x * res)
        new_msg.info.origin.position.y = msg.info.origin.position.y - (pad_px_y * res)
        new_msg.info.origin.position.z = msg.info.origin.position.z
        new_msg.info.origin.orientation = msg.info.origin.orientation
        
        new_data = [-1] * (new_width * new_height)
        
        for y in range(msg.info.height):
            for x in range(msg.info.width):
                old_idx = y * msg.info.width + x
                new_x = x + pad_px_x
                new_y = y + pad_px_y
                new_idx = new_y * new_width + new_x
                new_data[new_idx] = msg.data[old_idx]
                
        new_msg.data = new_data
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapPadder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()