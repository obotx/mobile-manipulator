#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped, PointStamped
from visualization_msgs.msg import Marker
from moveit_msgs.srv import ServoCommandType
from landmark_msgs.msg import LandmarkMsg, HandLandmark
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose

# ================= 🚨 EXACT CONFIGURATION FROM YOUR TF TREE 🚨 =================
PLANNING_FRAME = "odom_gt"
HAND_DATA_FRAME = "landmark"
BASE_FRAME = "obotx_base_link_platform"
FOOTPRINT_FRAME = "obotx_base_footprint_platform"
LEFT_ROOT_LINK = "obotx_left_arm_root_link"
RIGHT_ROOT_LINK = "obotx_right_arm_root_link"

CMD_VEL_TOPIC = "/cmd_vel"
USE_TWIST_STAMPED = True
# ===============================================================================

ALPHA = 0.3                  
MAX_STEP_PER_UPDATE = 0.02   

class ArmTracker:
    def __init__(self, node, side):
        self.state = "IDLE"

        self.side = side
        self.node = node
        self.pose_topic = f"/servo_node_{side}/pose_cmd"
        self.switch_service = f"/servo_node_{side}/switch_command_type"
        self.ee_frame = f"obotx_{side}_tool0"
        self.root_link = LEFT_ROOT_LINK if side == 'left' else RIGHT_ROOT_LINK
        
        self.pose_pub = node.create_publisher(PoseStamped, self.pose_topic, 10)
        self.switch_client = node.create_client(ServoCommandType, self.switch_service)
        self.marker_pub = node.create_publisher(Marker, f'/hand_target_marker_{side}', 10)
        
        self.is_tracking = False
        self.filtered_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.initialized_filter = False
        self.debug_counter = 0
        self.reachability_error = 0.0
        
        self.call_switch_service()

    def call_switch_service(self):
        self.node.get_logger().info(f"Waiting for {self.side.upper()} Servo switch service...")
        if not self.switch_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(f"❌ {self.side.upper()} Servo switch service not available!")
            return
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.POSE
        future = self.switch_client.call_async(request)
        future.add_done_callback(self.switch_callback)

    def switch_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"✅ Successfully switched {self.side.upper()} MoveIt Servo to POSE mode.")
        except Exception as e:
            self.node.get_logger().error(f"Service call failed for {self.side.upper()}: {e}")

    def get_current_ee_orientation(self):
        try:
            trans = self.node.tf_buffer.lookup_transform(PLANNING_FRAME, self.ee_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            return trans.transform.rotation
        except Exception:
            return None

    def transform_to_planning_frame(self, x, y, z):
        if HAND_DATA_FRAME == PLANNING_FRAME:
            return float(x), float(y), float(z)
        try:
            pose_in = PoseStamped()
            pose_in.header.frame_id = HAND_DATA_FRAME
            pose_in.header.stamp = rclpy.time.Time()
            pose_in.pose.position.x = float(x)
            pose_in.pose.position.y = float(y)
            pose_in.pose.position.z = float(z)
            pose_in.pose.orientation.w = 1.0 
            pose_out = self.node.tf_buffer.transform(pose_in, PLANNING_FRAME, timeout=Duration(seconds=0.1))
            return pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z
        except Exception as e:
            if self.debug_counter % 50 == 0:
                self.node.get_logger().warn(f"[{self.side.upper()}] TF transform failed. Error: {e}")
            return None, None, None

    def publish_target_marker(self, pose_msg: PoseStamped):
        try:
            root_tf = self.node.tf_buffer.lookup_transform(PLANNING_FRAME, self.root_link, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            root_x = root_tf.transform.translation.x
            root_y = root_tf.transform.translation.y
            root_z = root_tf.transform.translation.z
        except Exception:
            return  
        target_x = pose_msg.pose.position.x
        target_y = pose_msg.pose.position.y
        target_z = pose_msg.pose.position.z
        
        dist_xy = math.sqrt((target_x - root_x)**2 + (target_y - root_y)**2)
        dist_xyz = math.sqrt((target_x - root_x)**2 + (target_y - root_y)**2 + (target_z - root_z)**2)
        
        p_root = Point(x=root_x, y=root_y, z=root_z)
        p_corner = Point(x=target_x, y=target_y, z=root_z) 
        p_target = Point(x=target_x, y=target_y, z=target_z)
        
        line_marker = Marker()
        line_marker.header.frame_id = PLANNING_FRAME
        line_marker.header.stamp = pose_msg.header.stamp
        line_marker.ns = f"hand_target_line_{self.side}"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.01
        if self.side == 'left':
            line_marker.color.r = 0.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0
        else:
            line_marker.color.r = 0.0; line_marker.color.g = 0.5; line_marker.color.b = 1.0
        line_marker.color.a = 0.8 
        
        line_marker.points = [p_root, p_corner, p_target, p_root]
        self.marker_pub.publish(line_marker)

        text_marker = Marker()
        text_marker.header.frame_id = PLANNING_FRAME
        text_marker.header.stamp = pose_msg.header.stamp
        text_marker.ns = f"hand_target_text_{self.side}"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0
        text_marker.scale.z = 0.05  
        
        if self.side == 'left':
            text_marker.color.r = 0.0; text_marker.color.g = 1.0; text_marker.color.b = 0.0
        else:
            text_marker.color.r = 0.0; text_marker.color.g = 0.5; text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        # Position text right at the 90-degree corner, floating slightly above it
        text_marker.pose.position = Point(x=target_x, y=target_y, z=root_z + 0.05)
        text_marker.pose.orientation.w = 1.0
        
        text_marker.text = f"XY:{dist_xy:.2f}m\nXYZ:{dist_xyz:.2f}m"
        
        self.marker_pub.publish(text_marker)

    def update(self, hand_data: HandLandmark):
        workspace_recovery = (
            self.side == "left" and self.node.base_controller.left_workspace_recovery
        ) or (
            self.side == "right" and self.node.base_controller.right_workspace_recovery
        )

        self.debug_counter += 1
        if not hand_data.present:
            self.state = "NO_HAND"
            self.is_tracking = False
            self.reachability_error = 0.0
            return

        self.is_tracking = True
        raw_x, raw_y, raw_z = hand_data.wrist_m.x, hand_data.wrist_m.y, hand_data.wrist_m.z
        
        trans_x, trans_y, trans_z = self.transform_to_planning_frame(raw_x, raw_y, raw_z)
        if trans_x is None:
            return 
        
        try:
            root_tf = self.node.tf_buffer.lookup_transform(
                PLANNING_FRAME,
                self.root_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return

        root_x = root_tf.transform.translation.x
        root_y = root_tf.transform.translation.y

        dx = trans_x - root_x
        dy = trans_y - root_y
        dist_xy = math.sqrt(dx*dx + dy*dy)
        if workspace_recovery and dist_xy < self.node.base_controller.min_xy_distance:
            self.state = "WORKSPACE_RECOVERY"
            if dist_xy > 1e-6:
                scale = self.node.base_controller.min_xy_distance / dist_xy
                trans_x = root_x + dx * scale
                trans_y = root_y + dy * scale
                trans_z = trans_z
                        
        if not self.node.base_controller.freeze_arms:
            if not self.initialized_filter:
                self.filtered_pos['x'], self.filtered_pos['y'], self.filtered_pos['z'] = trans_x, trans_y, trans_z
                self.initialized_filter = True
            else:
                target_x = ALPHA * trans_x + (1.0 - ALPHA) * self.filtered_pos['x']
                target_y = ALPHA * trans_y + (1.0 - ALPHA) * self.filtered_pos['y']
                target_z = ALPHA * trans_z + (1.0 - ALPHA) * self.filtered_pos['z']
                
                dx, dy, dz = target_x - self.filtered_pos['x'], target_y - self.filtered_pos['y'], target_z - self.filtered_pos['z']
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > MAX_STEP_PER_UPDATE:
                    scale = MAX_STEP_PER_UPDATE / dist
                    dx *= scale; dy *= scale; dz *= scale
                    
                self.filtered_pos['x'] += dx
                self.filtered_pos['y'] += dy
                self.filtered_pos['z'] += dz
        else:
            self.state = "FROZEN"
            try:
                ee_tf = self.node.tf_buffer.lookup_transform(
                    PLANNING_FRAME,
                    self.ee_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1)
                )

                self.filtered_pos['x'] = ee_tf.transform.translation.x
                self.filtered_pos['y'] = ee_tf.transform.translation.y
                self.filtered_pos['z'] = ee_tf.transform.translation.z

            except Exception:
                pass

        current_rot = self.get_current_ee_orientation()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = PLANNING_FRAME
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=self.filtered_pos['x'], y=self.filtered_pos['y'], z=self.filtered_pos['z'])
        
        if current_rot:
            pose_msg.pose.orientation = Quaternion(x=current_rot.x, y=current_rot.y, z=current_rot.z, w=current_rot.w)
        else:
            pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
        self.pose_pub.publish(pose_msg)
        self.publish_target_marker(pose_msg)
        
        try:
            current_ee_tf = self.node.tf_buffer.lookup_transform(PLANNING_FRAME, self.ee_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
            curr_x, curr_y, curr_z = current_ee_tf.transform.translation.x, current_ee_tf.transform.translation.y, current_ee_tf.transform.translation.z
            self.reachability_error = math.sqrt((curr_x - self.filtered_pos['x'])**2 + (curr_y - self.filtered_pos['y'])**2 + (curr_z - self.filtered_pos['z'])**2)
        except Exception:
            self.reachability_error = 0.0

class BaseController:
    def __init__(self, node):
        self.state = "IDLE"

        self.node = node
        self.cmd_vel_topic = CMD_VEL_TOPIC
        
        self.cmd_vel_pub = node.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.node.get_logger().info(f"Base controller publishing TwistStamped to: {self.cmd_vel_topic}")
        
        self.kp_yaw = 0.8  
        self.smoothed_cmd = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.alpha_base = 0.15
        self.base_ready = True
        self.warn_counter = 0
        self.min_xy_distance = 0.40
        self.min_xy_release = 0.45
        self.reach_error_threshold = 0.15
        self.yaw_threshold_deg = 25.0
        self.yaw_threshold = math.radians(self.yaw_threshold_deg)
        self.max_assist_speed = 0.15
        self.max_yaw_speed = 0.5
        self.freeze_arms = False
        self.left_workspace_recovery = False
        self.right_workspace_recovery = False
        self.filtered_human_yaw = None
        self.yaw_alpha = 0.15

    def calc_xy_distance(self, target_pos, arm_root_link):
        try:
            # Increased timeout to 1.0s to ensure TF buffer is fully populated
            root_tf = self.node.tf_buffer.lookup_transform(
                PLANNING_FRAME,
                arm_root_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            rx = root_tf.transform.translation.x
            ry = root_tf.transform.translation.y
            dx = target_pos['x'] - rx
            dy = target_pos['y'] - ry
            dist_xy = math.sqrt(dx*dx + dy*dy)
            return dist_xy, dx, dy
        except Exception as e:
            # Fixed: use self.warn_counter, not self.node.warn_counter
            if self.warn_counter % 50 == 0:
                self.node.get_logger().warning(f"TF lookup failed for {arm_root_link}. Error: {e}")
            return None  # Safely return None to prevent unpacking crashes
    
    def update(self, msg: LandmarkMsg, left_target=None, right_target=None, left_error=0.0, right_error=0.0):
        self.warn_counter += 1
        
        body_data = {bl.joint_name: [bl.x, bl.y, bl.z] for bl in msg.body_landmarks}
        if "shoulder_L" not in body_data or "shoulder_R" not in body_data:
            self.state = "NO_HUMAN"
            return
        
        try:
            sL, sR = body_data["shoulder_L"], body_data["shoulder_R"]
            left_shoulder = PointStamped()
            left_shoulder.header.frame_id = HAND_DATA_FRAME
            left_shoulder.point.x = sL[0]; left_shoulder.point.y = sL[1]; left_shoulder.point.z = sL[2]

            right_shoulder = PointStamped()
            right_shoulder.header.frame_id = HAND_DATA_FRAME
            right_shoulder.point.x = sR[0]; right_shoulder.point.y = sR[1]; right_shoulder.point.z = sR[2]

            left_odom = self.node.tf_buffer.transform(left_shoulder, PLANNING_FRAME, timeout=Duration(seconds=0.1))
            right_odom = self.node.tf_buffer.transform(right_shoulder, PLANNING_FRAME, timeout=Duration(seconds=0.1))

            sx = right_odom.point.x - left_odom.point.x
            sy = right_odom.point.y - left_odom.point.y
            forward_x, forward_y = -sy, sx
            human_yaw_raw = math.atan2(forward_y, forward_x)

            if self.filtered_human_yaw is None:
                self.filtered_human_yaw = human_yaw_raw
            else:
                yaw_diff = (human_yaw_raw - self.filtered_human_yaw + math.pi) % (2 * math.pi) - math.pi
                self.filtered_human_yaw = (self.filtered_human_yaw + self.yaw_alpha * yaw_diff + math.pi) % (2 * math.pi) - math.pi

            human_yaw = self.filtered_human_yaw
            cmd = TwistStamped()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.header.frame_id = FOOTPRINT_FRAME
            
            try:
                base_tf = self.node.tf_buffer.lookup_transform(PLANNING_FRAME, FOOTPRINT_FRAME, rclpy.time.Time(), timeout=Duration(seconds=0.1))
                q = base_tf.transform.rotation
                robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                yaw_error = (human_yaw - robot_yaw + math.pi) % (2*math.pi) - math.pi

                if abs(yaw_error) > self.yaw_threshold:
                    self.state = "ROTATING_TO_HUMAN"
                    self.freeze_arms = True
                    cmd.twist.angular.z = max(-self.max_yaw_speed, min(self.max_yaw_speed, self.kp_yaw * yaw_error))
                else:
                    self.freeze_arms = False
                    cmd.twist.angular.z = max(-0.15, min(0.15, self.kp_yaw * yaw_error))
            except Exception:
                cmd.twist.angular.z = 0.0

            v_x, v_y = 0.0, 0.0

            # --- SAFE LEFT ARM CHECK ---
            if left_target is not None:
                result = self.calc_xy_distance(left_target, LEFT_ROOT_LINK)
                if result is not None:
                    dist_xy, dx, dy = result
                    if not self.left_workspace_recovery:
                        if dist_xy < self.min_xy_distance: self.left_workspace_recovery = True
                    else:
                        if dist_xy > self.min_xy_release: self.left_workspace_recovery = False

                    norm = max(0.01, dist_xy)
                    if self.left_workspace_recovery:
                        v_x += -(dx / norm) * self.max_assist_speed
                        v_y += -(dy / norm) * self.max_assist_speed
                    elif left_error > self.reach_error_threshold:
                        v_x += (dx / norm) * 0.08
                        v_y += (dy / norm) * 0.08
            
            # --- SAFE RIGHT ARM CHECK ---
            if right_target is not None:
                result = self.calc_xy_distance(right_target, RIGHT_ROOT_LINK)
                if result is not None:
                    dist_xy, dx, dy = result
                    if not self.right_workspace_recovery:
                        if dist_xy < self.min_xy_distance: self.right_workspace_recovery = True
                    else:
                        if dist_xy > self.min_xy_release: self.right_workspace_recovery = False

                    norm = max(0.01, dist_xy)
                    if self.right_workspace_recovery:
                        v_x += -(dx / norm) * self.max_assist_speed
                        v_y += -(dy / norm) * self.max_assist_speed
                    elif right_error > self.reach_error_threshold:
                        v_x += (dx / norm) * 0.08
                        v_y += (dy / norm) * 0.08

            mag = math.sqrt(v_x*v_x + v_y*v_y)
            if mag > self.max_assist_speed:
                scale = self.max_assist_speed / mag
                v_x *= scale; v_y *= scale

            cmd.twist.linear.x = v_x
            cmd.twist.linear.y = v_y
            
            self.smoothed_cmd['x'] = self.alpha_base * cmd.twist.linear.x + (1 - self.alpha_base) * self.smoothed_cmd['x']
            self.smoothed_cmd['y'] = self.alpha_base * cmd.twist.linear.y + (1 - self.alpha_base) * self.smoothed_cmd['y']
            self.smoothed_cmd['yaw'] = self.alpha_base * cmd.twist.angular.z + (1 - self.alpha_base) * self.smoothed_cmd['yaw']
            
            cmd.twist.linear.x = max(-0.3, min(0.3, self.smoothed_cmd['x']))
            cmd.twist.linear.y = max(-0.3, min(0.3, self.smoothed_cmd['y']))
            cmd.twist.angular.z = max(-0.5, min(0.5, self.smoothed_cmd['yaw']))
            
            if self.left_workspace_recovery or self.right_workspace_recovery:
                self.state = "WORKSPACE_RECOVERY"
            elif (
                left_error > self.reach_error_threshold
                or right_error > self.reach_error_threshold
            ):
                self.state = "REACHABILITY_ASSIST"
            else:
                self.state = "TRACKING"

            if abs(cmd.twist.linear.x) > 0.01 or abs(cmd.twist.linear.y) > 0.01 or abs(cmd.twist.angular.z) > 0.05:
                self.cmd_vel_pub.publish(cmd)
            else:
                stop_cmd = TwistStamped()
                stop_cmd.header.stamp = self.node.get_clock().now().to_msg()
                stop_cmd.header.frame_id = FOOTPRINT_FRAME
                self.cmd_vel_pub.publish(stop_cmd)
                
        except Exception as e:
            self.node.get_logger().error(f"Base controller update failed: {e}")

class HandPoseServoNode(Node):
    def __init__(self):
        super().__init__('hand_pose_tracker')
        self.declare_parameter('closest_target', True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.left_tracker = ArmTracker(self, 'left')
        self.right_tracker = ArmTracker(self, 'right')
        self.base_controller = BaseController(self)
        self.create_timer(1.0, self.log_states)

        self.human_yaw_marker_pub = self.create_publisher(Marker, '/human_yaw_marker', 10)
        self.landmark_sub = self.create_subscription(LandmarkMsg, '/processed_landmarks', self.landmark_callback, 10)
        self.get_logger().info("✅ Dual-Arm Hand Pose Tracker + Base Optimizer Initialized.")

    def log_states(self):
        self.get_logger().info(
            f"BASE={self.base_controller.state} | "
            f"LEFT={self.left_tracker.state} | "
            f"RIGHT={self.right_tracker.state} | "
            f"Lerr={self.left_tracker.reachability_error:.2f} | "
            f"Rerr={self.right_tracker.reachability_error:.2f}"
        )

    def publish_human_yaw_marker(self, msg: LandmarkMsg):
        body_data = {bl.joint_name: [bl.x, bl.y, bl.z] for bl in msg.body_landmarks}
        if "shoulder_L" not in body_data or "shoulder_R" not in body_data:
            return
        
        try:
            sL, sR = body_data["shoulder_L"], body_data["shoulder_R"]
            
            left_shoulder = PointStamped()
            left_shoulder.header.frame_id = HAND_DATA_FRAME
            left_shoulder.point.x = float(sL[0])
            left_shoulder.point.y = float(sL[1])
            left_shoulder.point.z = float(sL[2])

            right_shoulder = PointStamped()
            right_shoulder.header.frame_id = HAND_DATA_FRAME
            right_shoulder.point.x = float(sR[0])
            right_shoulder.point.y = float(sR[1])
            right_shoulder.point.z = float(sR[2])

            # Transform shoulders to planning frame
            left_odom = self.tf_buffer.transform(left_shoulder, PLANNING_FRAME, timeout=Duration(seconds=0.1))
            right_odom = self.tf_buffer.transform(right_shoulder, PLANNING_FRAME, timeout=Duration(seconds=0.1))

            # Vector from left to right shoulder
            sx = right_odom.point.x - left_odom.point.x
            sy = right_odom.point.y - left_odom.point.y
            
            # Normalize to ensure consistent arrow length regardless of distance
            mag_shoulder = math.sqrt(sx*sx + sy*sy)
            if mag_shoulder > 0.1:
                sx_norm = sx / mag_shoulder
                sy_norm = sy / mag_shoulder
            else:
                sx_norm, sy_norm = 0.0, 0.0

            # Forward vector is perpendicular to the shoulder line
            forward_x = -sy_norm
            forward_y = sx_norm
            human_yaw_raw = math.atan2(forward_y, forward_x)
            
            # Center point between shoulders
            center_x = (left_odom.point.x + right_odom.point.x) / 2.0
            center_y = (left_odom.point.y + right_odom.point.y) / 2.0
            center_z = (left_odom.point.z + right_odom.point.z) / 2.0
            
            arrow_length = 0.6  # meters
            
            # --- 1. Draw Arrow ---
            arrow_marker = Marker()
            arrow_marker.header.frame_id = PLANNING_FRAME
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = "human_yaw_arrow"
            arrow_marker.id = 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            p_start = Point(x=center_x, y=center_y, z=center_z)
            p_end = Point(x=center_x + forward_x * arrow_length, 
                          y=center_y + forward_y * arrow_length, 
                          z=center_z)
            arrow_marker.points = [p_start, p_end]
            
            arrow_marker.scale.x = 0.05  # shaft diameter
            arrow_marker.scale.y = 0.1   # arrowhead diameter
            arrow_marker.scale.z = 0.0   # arrowhead length (0 = default)
            arrow_marker.color.r = 1.0; arrow_marker.color.g = 0.5; arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0   # Orange
            
            self.human_yaw_marker_pub.publish(arrow_marker)

            # --- 2. Draw Text ---
            text_marker = Marker()
            text_marker.header.frame_id = PLANNING_FRAME
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "human_yaw_text"
            text_marker.id = 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.05
            text_marker.color.r = 1.0; text_marker.color.g = 0.0; text_marker.color.b = 0.0
            text_marker.color.a = 1.0    # Yellow
            
            text_marker.pose.position = Point(x=center_x, y=center_y, z=center_z)
            text_marker.pose.orientation.w = 1.0
            
            yaw_deg = math.degrees(human_yaw_raw)
            text_marker.text = f"Human:{yaw_deg:.1f}°"
            
            self.human_yaw_marker_pub.publish(text_marker)

        except Exception:
            pass # Silently ignore TF errors during visualization to prevent spam

    def landmark_callback(self, msg: LandmarkMsg):
        closest_target = self.get_parameter('closest_target').value
        
        try:
            # Increased timeout to 1.0s here as well
            left_root_tf = self.tf_buffer.lookup_transform(HAND_DATA_FRAME, LEFT_ROOT_LINK, rclpy.time.Time(), timeout=Duration(seconds=1.0))
            right_root_tf = self.tf_buffer.lookup_transform(HAND_DATA_FRAME, RIGHT_ROOT_LINK, rclpy.time.Time(), timeout=Duration(seconds=1.0))
            
            left_root_pos = (left_root_tf.transform.translation.x, left_root_tf.transform.translation.y, left_root_tf.transform.translation.z)
            right_root_pos = (right_root_tf.transform.translation.x, right_root_tf.transform.translation.y, right_root_tf.transform.translation.z)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to get arm root TF. Error: {e}")
            return

        def get_dist(hand_data, root_pos):
            return math.sqrt((hand_data.wrist_m.x - root_pos[0])**2 + (hand_data.wrist_m.y - root_pos[1])**2 + (hand_data.wrist_m.z - root_pos[2])**2)

        empty_hand = HandLandmark()
        empty_hand.present = False

        if not closest_target:
            self.left_tracker.update(msg.left_hand)
            self.right_tracker.update(msg.right_hand)
        else:
            hands = []
            if msg.left_hand.present: hands.append(('left', msg.left_hand))
            if msg.right_hand.present: hands.append(('right', msg.right_hand))
            
            if len(hands) == 0:
                self.left_tracker.update(empty_hand)
                self.right_tracker.update(empty_hand)
            elif len(hands) == 1:
                hand_name, hand_data = hands[0]
                dist_to_left = get_dist(hand_data, left_root_pos)
                dist_to_right = get_dist(hand_data, right_root_pos)
                if dist_to_left < dist_to_right:
                    self.left_tracker.update(hand_data)
                    self.right_tracker.update(empty_hand)
                else:
                    self.right_tracker.update(hand_data)
                    self.left_tracker.update(empty_hand)
            elif len(hands) == 2:
                h_left = hands[0][1] if hands[0][0] == 'left' else hands[1][1]
                h_right = hands[0][1] if hands[0][0] == 'right' else hands[1][1]
                cost_straight = get_dist(h_left, left_root_pos) + get_dist(h_right, right_root_pos)
                cost_cross = get_dist(h_left, right_root_pos) + get_dist(h_right, left_root_pos)
                if cost_cross < cost_straight:
                    self.right_tracker.update(h_left)
                    self.left_tracker.update(h_right)
                else:
                    self.left_tracker.update(h_left)
                    self.right_tracker.update(h_right)

        left_target = self.left_tracker.filtered_pos if self.left_tracker.is_tracking else None
        right_target = self.right_tracker.filtered_pos if self.right_tracker.is_tracking else None
        left_error = self.left_tracker.reachability_error if self.left_tracker.is_tracking else 0.0
        right_error = self.right_tracker.reachability_error if self.right_tracker.is_tracking else 0.0
        self.publish_human_yaw_marker(msg)
        # self.base_controller.update(msg, left_target, right_target, left_error, right_error)


def main():
    rclpy.init()
    node = HandPoseServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()