#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class OctoHandler(Node):

    def __init__(self):
        super().__init__('moveit_octomap_handler')
        self.map_msg = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        # Subscriber
        self.subscription = self.create_subscription(
            Octomap,
            '/octomap_full',
            self.cb,
            qos
        )
        # Publishers
        self.pub_monitored = self.create_publisher(
            PlanningScene,
            '/monitored_planning_scene',
            qos
        )

        self.pub_planning = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            qos
        )
        # Timer (0.25 Hz = every 4 seconds)
        self.timer = self.create_timer(4.0, self.publish_scene)
        self.get_logger().info('MoveIt Octomap handler started.')

    def publish_scene(self):
        if self.map_msg is not None:
            self.pub_monitored.publish(self.map_msg)
            self.pub_planning.publish(self.map_msg)

    def cb(self, msg: Octomap):
        psw = PlanningSceneWorld()
        # Octomap message
        psw.octomap.header.stamp = self.get_clock().now().to_msg()
        psw.octomap.header.frame_id = 'map'
        psw.octomap.octomap = msg
        # Origin pose
        origin = Pose()
        origin.position.x = 0.0
        origin.position.y = 0.0
        origin.position.z = 0.0
        origin.orientation.x = 0.0
        origin.orientation.y = 0.0
        origin.orientation.z = 0.0
        origin.orientation.w = 1.0
        psw.octomap.origin = origin
        # Planning scene
        ps = PlanningScene()
        ps.world = psw
        ps.is_diff = True

        self.map_msg = ps


def main(args=None):
    rclpy.init(args=args)
    node = OctoHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()