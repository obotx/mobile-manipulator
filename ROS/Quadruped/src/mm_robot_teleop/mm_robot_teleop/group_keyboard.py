#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import copy
from pynput import keyboard

class CartesianTeleopNode(Node):
    def __init__(self):
        super().__init__('morph_i_cartesian_teleop')
        
        # Initialize MoveItPy
        self.get_logger().info("Initializing MoveItPy...")
        self.moveit = MoveItPy(node_name="moveit_py")
        self.get_logger().info("MoveItPy initialized")
        
        # Get planning components for both arms
        self.left_arm = self.moveit.get_planning_component("left_arm")
        self.right_arm = self.moveit.get_planning_component("right_arm")
        
        # Default to right arm
        self.active_arm_name = "right_arm"
        self.active_arm = self.right_arm
        self.step_size = 0.01  # 1 cm per key press
        
        # TF2 for pose lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # End-effector link names (adjust to your URDF)
        self.ee_links = {
            "left_arm": "left_end_effector",   # <-- CHANGE TO YOUR LINK NAME
            "right_arm": "right_end_effector" # <-- CHANGE TO YOUR LINK NAME
        }
        self.base_frame = "odom_gt"
        
        # Initialize target pose from current end-effector pose
        self.target_pose = self._get_current_pose()
        
        self.get_logger().info("=== Morph-I Cartesian Teleop (ROS 2 Jazzy + moveit_py) ===")
        self.get_logger().info("Keys: W/S -> +X/-X | A/D -> +Y/-Y | Q/E -> +Z/-Z")
        self.get_logger().info("Press 'L' for Left Arm | 'R' for Right Arm")
        self.get_logger().info("Ctrl+C to exit\n")
        
        # Keyboard state tracking
        self.key_states = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False}
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        # 30 Hz control loop
        self.timer = self.create_timer(0.033, self.control_loop)

    def _get_current_pose(self):
        """Get current end-effector pose as PoseStamped via TF2."""
        ee_link = self.ee_links[self.active_arm_name]
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        
        try:
            # Lookup transform from base_frame to end-effector
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            # Return zero pose as fallback
            pose.pose.orientation.w = 1.0
            
        return pose

    def on_press(self, key):
        try:
            k = key.char.lower()
            if k in self.key_states:
                self.key_states[k] = True
            elif k == 'l':
                self._switch_arm("left_arm")
            elif k == 'r':
                self._switch_arm("right_arm")
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            k = key.char.lower()
            if k in self.key_states:
                self.key_states[k] = False
        except AttributeError:
            pass

    def _switch_arm(self, name):
        self.active_arm_name = name
        self.active_arm = self.moveit.get_planning_component(name)
        self.target_pose = self._get_current_pose()
        self.get_logger().info(f"Switched to {name} arm")

    def control_loop(self):
        dx = self.step_size * (self.key_states['w'] - self.key_states['s'])
        dy = self.step_size * (self.key_states['a'] - self.key_states['d'])
        dz = self.step_size * (self.key_states['q'] - self.key_states['e'])

        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            # Update target pose
            self.target_pose.pose.position.x += dx
            self.target_pose.pose.position.y += dy
            self.target_pose.pose.position.z += dz

            # Plan to target pose using moveit_py
            try:
                # Set start state to current state
                self.active_arm.set_start_state_to_current_state()
                
                # Set pose goal
                pose_goal = PoseStamped()
                pose_goal.header = self.target_pose.header
                pose_goal.pose = self.target_pose.pose
                
                self.active_arm.set_goal_state(
                    pose_stamped_msg=pose_goal, 
                    pose_link=self.ee_links[self.active_arm_name]
                )
                
                # Plan
                plan_result = self.active_arm.plan()
                
                # Execute if plan succeeded
                if plan_result and plan_result[0]:  # plan_result returns (success, trajectory_message)
                    trajectory = plan_result[1]
                    self.moveit.execute(trajectory, controllers=[])
                else:
                    self.get_logger().warn_throttle(1.0, "Cartesian path blocked or invalid!")
                    
            except Exception as e:
                self.get_logger().warn_throttle(1.0, f"Planning/execution failed: {e}")

    def destroy_node(self):
        self.listener.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CartesianTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()