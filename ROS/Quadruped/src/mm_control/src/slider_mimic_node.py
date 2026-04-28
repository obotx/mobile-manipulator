#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class ClosedLoopMimicNode(Node):
    def __init__(self):
        super().__init__('closed_loop_mimic_node')
        
        # Kinematic Constants
        self.declare_parameter('k', 0.165966) # Your specific geometric constant
        self.declare_parameter('hinge_clamp_min', -1.4)
        self.declare_parameter('hinge_clamp_max', 1.4)
        self.declare_parameter('slide_min', 0.0)
        self.declare_parameter('slide_max', 1.112)
        
        # 🚀 CRITICAL: Run at 1000Hz (1ms) to match Gazebo default physics step
        # This ensures the command arrives before the physics solver calculates the next step
        self.update_rate = 1000.0 
        
        self.k = self.get_parameter('k').value
        self.hinge_min = self.get_parameter('hinge_clamp_min').value
        self.hinge_max = self.get_parameter('hinge_clamp_max').value
        self.slide_min = self.get_parameter('slide_min').value
        self.slide_max = self.get_parameter('slide_max').value
        
        self.left_slide = 0.0
        self.left_hinge = 0.0
        self.state_ready = False
        
        # Low queue size (1) ensures we always process the freshest state
        self.sub = self.create_subscription(JointState, '/joint_states', self.state_cb, 1)
        
        # Publish to the forward controller
        self.pub = self.create_publisher(Float64MultiArray, '/right_side_controller/commands', 1)
        
        # Timer runs at 1000Hz
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
        
        self.get_logger().info('🔒 Closed-Loop Mimic Active: Snapping right slider at 1000Hz')

    def state_cb(self, msg: JointState):
        try:
            # Get current state of the LEFT side (the driver)
            self.left_slide = msg.position[msg.name.index('joint_slider_left_slide')]
            self.left_hinge = msg.position[msg.name.index('joint_slider_left_hinge')]
            self.state_ready = True
        except ValueError:
            pass

    def control_loop(self):
        if not self.state_ready:
            return

        # 1. Clamp hinge to avoid singularity at +/- 90 degrees
        hinge_clamped = max(self.hinge_min, min(self.hinge_max, self.left_hinge))
        
        # 2. Calculate Geometry: Right = Left_Slide - K * tan(Left_Hinge)
        # This enforces the rigid triangle constraint
        target = self.left_slide - self.k * math.tan(hinge_clamped)
        
        # 3. Hard clamp to physical limits (prevent breaking the model)
        target = max(self.slide_min, min(self.slide_max, target))
        
        # 4. 🚀 INSTANT COMMAND: No threshold check. 
        # We send the target EVERY cycle so the physics engine sees no error.
        msg = Float64MultiArray()
        msg.data = [target]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopMimicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()