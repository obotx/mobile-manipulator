#!/usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import cv2
import mediapipe as mp

POSE_CONNECTIONS = [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]
BODY_LANDMARKS = [11, 12, 13, 14, 15, 16]
HAND_CONNECTIONS = list(mp.solutions.hands.HAND_CONNECTIONS)
FINGERTIP_INDICES = [4, 8, 12, 16, 20]

class MediaPipeBodyNode(Node):

    def __init__(self):
        super().__init__('mediapipe_body_node')

        self.declare_parameter('arm_length_scale', 1.0)
        self.arm_length_scale = self.get_parameter('arm_length_scale').value
        
        self.declare_parameter('target_frame', 'obotx_base_footprint_platform')
        self.target_frame = self.get_parameter('target_frame').value

        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.offset_z = self.get_parameter('offset_z').value

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.compressed_callback, qos_profile=qos_img
        )
        self.landmark_pub = self.create_publisher(MarkerArray, '/body_landmarks', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.publish_static_transform()

        self.mp_pose = mp.solutions.pose.Pose(
            model_complexity=0, smooth_landmarks=True,
            min_detection_confidence=0.5, min_tracking_confidence=0.5
        )
        self.mp_hands = mp.solutions.hands.Hands(
            max_num_hands=2, min_detection_confidence=0.3, min_tracking_confidence=0.3
        )

        self.last_hand_results = None
        self.process_hands_every = 1
        self._frame_count = 0
        self.prev_pose_world = [None] * 33
        self.pose_alpha = 0.3

        self.quat_alpha = 0.35
        self.prev_quats = {'left': None, 'right': None}
        self.prev_hand_wrist_pos = {'left': None, 'right': None}

        self.get_logger().info(f"MediaPipe node started. Target: '{self.target_frame}', Offset: ({self.offset_x}, {self.offset_y}, {self.offset_z})")

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = Time().to_msg() 
        static_transform.header.frame_id = self.target_frame
        static_transform.child_frame_id = 'landmark'
        static_transform.transform.translation.x = float(self.offset_x)
        static_transform.transform.translation.y = float(self.offset_y)
        static_transform.transform.translation.z = float(self.offset_z)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f"Published static transform: '{self.target_frame}' -> 'landmark'")


    def apply_ema_filter(self, landmarks):
        alpha = self.pose_alpha
        if self.prev_pose_world[0] is None:
            self.prev_pose_world = [(lm.x, lm.y, lm.z) for lm in landmarks]
            return landmarks
        for i, lm in enumerate(landmarks):
            px, py, pz = self.prev_pose_world[i]
            lm.x = alpha * lm.x + (1.0 - alpha) * px
            lm.y = alpha * lm.y + (1.0 - alpha) * py
            lm.z = alpha * lm.z + (1.0 - alpha) * pz
            self.prev_pose_world[i] = (lm.x, lm.y, lm.z)
        return landmarks

    def create_line_marker(self, ns, marker_id, landmarks, connections, color, transform_fn, line_width=0.005, use_visibility=False, lifetime_sec=0.1, stamp=None):
        marker = Marker()
        marker.header.frame_id = 'landmark'
        marker.header.stamp = stamp if stamp else self.get_clock().now().to_msg()
        marker.ns = ns; marker.id = marker_id
        marker.type = Marker.LINE_LIST; marker.action = Marker.ADD
        marker.scale.x = line_width; marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.lifetime = Duration(seconds=lifetime_sec).to_msg()
        for start, end in connections:
            if use_visibility and (landmarks[start].visibility < 0.5 or landmarks[end].visibility < 0.5): continue
            marker.points.append(transform_fn(landmarks[start]))
            marker.points.append(transform_fn(landmarks[end]))
        return marker

    def create_points_marker(self, ns, marker_id, landmarks, indices, color, transform_fn, point_size=0.015, use_visibility=False, lifetime_sec=0.1, stamp=None):
        marker = Marker()
        marker.header.frame_id = 'landmark'
        marker.header.stamp = stamp if stamp else self.get_clock().now().to_msg()
        marker.ns = ns; marker.id = marker_id
        marker.type = Marker.POINTS; marker.action = Marker.ADD
        marker.scale.x, marker.scale.y = point_size, point_size
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.lifetime = Duration(seconds=lifetime_sec).to_msg()
        for idx in indices:
            lm = landmarks[idx]
            if use_visibility and lm.visibility < 0.5: continue
            marker.points.append(transform_fn(lm))
        return marker

    def _mp_vec_to_ros(self, v):
        return np.array([-v[2], -v[0], -v[1]])

    def _mat_to_quaternion(self, m):
        trace = m[0,0] + m[1,1] + m[2,2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            qw, qx, qy, qz = 0.25/s, (m[2,1]-m[1,2])*s, (m[0,2]-m[2,0])*s, (m[1,0]-m[0,1])*s
        elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
            s = 2.0 * math.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2])
            qw, qx, qy, qz = (m[2,1]-m[1,2])/s, 0.25*s, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s
        elif m[1,1] > m[2,2]:
            s = 2.0 * math.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2])
            qw, qx, qy, qz = (m[0,2]-m[2,0])/s, (m[0,1]+m[1,0])/s, 0.25*s, (m[1,2]+m[2,1])/s
        else:
            s = 2.0 * math.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1])
            qw, qx, qy, qz = (m[1,0]-m[0,1])/s, (m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s, 0.25*s
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        return Quaternion(x=qx/norm, y=qy/norm, z=qz/norm, w=qw/norm)

    def compute_hand_quaternion(self, hand_lms, side):
        w = np.array([hand_lms[0].x, hand_lms[0].y, hand_lms[0].z])
        mid_mcp = np.array([hand_lms[9].x, hand_lms[9].y, hand_lms[9].z])
        thumb_mcp = np.array([hand_lms[2].x, hand_lms[2].y, hand_lms[2].z])

        x_mp = mid_mcp - w
        norm_x = np.linalg.norm(x_mp)
        x_mp = x_mp / norm_x if norm_x > 1e-6 else np.array([1.0, 0.0, 0.0])

        y_raw = thumb_mcp - w
        y_mp = y_raw - np.dot(y_raw, x_mp) * x_mp
        norm_y = np.linalg.norm(y_mp)
        y_mp = y_mp / norm_y if norm_y > 1e-6 else np.array([0.0, 1.0, 0.0])

        z_mp = np.cross(y_mp, x_mp)
        norm_z = np.linalg.norm(z_mp)
        z_mp = z_mp / norm_z if norm_z > 1e-6 else np.array([0.0, 0.0, 1.0])

        x_ros = self._mp_vec_to_ros(x_mp)
        y_ros = self._mp_vec_to_ros(y_mp)
        z_ros = self._mp_vec_to_ros(z_mp)

        R = np.column_stack((x_ros, y_ros, z_ros))
        if side == 'left':
            R_x_180 = np.array([[1.0,  0.0,  0.0], [0.0, -1.0,  0.0], [0.0,  0.0, -1.0]])
            R = R @ R_x_180

        return self._mat_to_quaternion(R)

    def smooth_quaternion(self, side, q_new):
        if self.prev_quats[side] is None:
            self.prev_quats[side] = q_new
            return q_new
        q_prev = self.prev_quats[side]
        
        dot = q_prev.x*q_new.x + q_prev.y*q_new.y + q_prev.z*q_new.z + q_prev.w*q_new.w
        if dot < 0:
            q_prev = Quaternion(x=-q_prev.x, y=-q_prev.y, z=-q_prev.z, w=-q_prev.w)
            dot = -dot
            
        dot = max(min(dot, 1.0), -1.0)
        theta = math.acos(dot)
        if theta < 1e-6:
            return q_new
            
        sin_t = math.sin(theta)
        a = math.sin((1.0 - self.quat_alpha) * theta) / sin_t
        b = math.sin(self.quat_alpha * theta) / sin_t
        
        q_smooth = Quaternion(
            x=a*q_prev.x + b*q_new.x,
            y=a*q_prev.y + b*q_new.y,
            z=a*q_prev.z + b*q_new.z,
            w=a*q_prev.w + b*q_new.w
        )
        self.prev_quats[side] = q_smooth
        return q_smooth

    def compressed_callback(self, msg):
        self._frame_count += 1
        marker_stamp = Time().to_msg()
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            tf_stamp = msg.header.stamp
        else:
            tf_stamp = (self.get_clock().now() - Duration(seconds=0.2)).to_msg()

        try:
            cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if cv_image is None: return
        except Exception as e:
            self.get_logger().error(f"Decode failed: {e}"); return

        cv_image = cv2.resize(cv_image, (640, 480))
        cv_image = cv2.flip(cv_image, 1)
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        pose_results = self.mp_pose.process(rgb)
        if self._frame_count % self.process_hands_every == 0:
            self.last_hand_results = self.mp_hands.process(rgb)
        hand_results = self.last_hand_results

        marker_array = MarkerArray()
        current_id = 0

        if pose_results.pose_world_landmarks and pose_results.pose_landmarks:
            pose_world = list(pose_results.pose_world_landmarks.landmark)
            pose_world = self.apply_ema_filter(pose_world)

            if self.arm_length_scale != 1.0:
                for idx in [13, 15]:
                    pose_world[idx].x = pose_world[11].x + (pose_world[idx].x - pose_world[11].x) * self.arm_length_scale
                    pose_world[idx].y = pose_world[11].y + (pose_world[idx].y - pose_world[11].y) * self.arm_length_scale
                    pose_world[idx].z = pose_world[11].z + (pose_world[idx].z - pose_world[11].z) * self.arm_length_scale
                for idx in [14, 16]:
                    pose_world[idx].x = pose_world[12].x + (pose_world[idx].x - pose_world[12].x) * self.arm_length_scale
                    pose_world[idx].y = pose_world[12].y + (pose_world[idx].y - pose_world[12].y) * self.arm_length_scale
                    pose_world[idx].z = pose_world[12].z + (pose_world[idx].z - pose_world[12].z) * self.arm_length_scale

            Sx = (pose_world[11].x + pose_world[12].x) / 2.0
            Sy = (pose_world[11].y + pose_world[12].y) / 2.0
            Sz = (pose_world[11].z + pose_world[12].z) / 2.0
            shoulder_offset = (Sx, Sy, Sz)

            def body_transform(lm):
                p = Point()
                p.x = -(lm.z - shoulder_offset[2]); p.y = -(lm.x - shoulder_offset[0]); p.z = -(lm.y - shoulder_offset[1])
                return p

            marker_array.markers.append(self.create_line_marker('body_edges', current_id, pose_world, POSE_CONNECTIONS, (0.0, 1.0, 0.0), body_transform, line_width=0.01, lifetime_sec=0.1, stamp=marker_stamp))
            current_id += 1
            marker_array.markers.append(self.create_points_marker('body_joints', current_id, pose_world, BODY_LANDMARKS, (0.0, 1.0, 0.0), body_transform, point_size=0.01, lifetime_sec=0.1, stamp=marker_stamp))
            current_id += 1

            if hand_results and hand_results.multi_hand_world_landmarks:
                current_hands = []
                for i in range(len(hand_results.multi_hand_world_landmarks)):
                    hand_world = hand_results.multi_hand_world_landmarks[i]
                    wrist_lm = hand_world.landmark[0]
                    wrist_pos = np.array([wrist_lm.x, wrist_lm.y, wrist_lm.z])
                    label = hand_results.multi_handedness[i].classification[0].label
                    
                    current_hands.append({
                        'landmarks': hand_world.landmark,
                        'wrist_pos': wrist_pos,
                        'mp_label': 'left' if label == 'Left' else 'right'
                    })

                matched_hands = {'left': None, 'right': None}
                left_candidates = [h for h in current_hands if h['mp_label'] == 'left']
                right_candidates = [h for h in current_hands if h['mp_label'] == 'right']

                if left_candidates:
                    if self.prev_hand_wrist_pos['left'] is not None:
                        matched_hands['left'] = min(left_candidates, key=lambda h: np.linalg.norm(h['wrist_pos'] - self.prev_hand_wrist_pos['left']))
                    else:
                        matched_hands['left'] = left_candidates[0]

                if right_candidates:
                    if self.prev_hand_wrist_pos['right'] is not None:
                        matched_hands['right'] = min(right_candidates, key=lambda h: np.linalg.norm(h['wrist_pos'] - self.prev_hand_wrist_pos['right']))
                    else:
                        matched_hands['right'] = right_candidates[0]

                for side in ['left', 'right']:
                    if matched_hands[side] is None:
                        self.prev_hand_wrist_pos[side] = None
                        continue

                    hand_lms = matched_hands[side]['landmarks']
                    self.prev_hand_wrist_pos[side] = matched_hands[side]['wrist_pos']

                    body_wrist_idx = 15 if side == "left" else 16
                    body_wrist = pose_world[body_wrist_idx]
                    hand_wrist_mp = hand_lms[0]

                    dx = body_wrist.x - hand_wrist_mp.x
                    dy = body_wrist.y - hand_wrist_mp.y
                    dz = body_wrist.z - hand_wrist_mp.z

                    def hand_transform(lm, dx=dx, dy=dy, dz=dz, offset=shoulder_offset):
                        x = lm.x + dx; y = lm.y + dy; z = lm.z + dz
                        p = Point()
                        p.x = -(z - offset[2]); p.y = -(x - offset[0]); p.z = -(y - offset[1])
                        return p

                    # Markers use marker_stamp (Time 0)
                    marker_array.markers.append(self.create_line_marker(f'hand_{side}_edges', current_id, hand_lms, HAND_CONNECTIONS, (1.0, 0.0, 0.0), hand_transform, 0.007, lifetime_sec=0.1, stamp=marker_stamp))
                    current_id += 1
                    marker_array.markers.append(self.create_points_marker(f'hand_{side}_joints', current_id, hand_lms, list(range(21)), (1.0, 0.0, 0.0), hand_transform, point_size=0.01, lifetime_sec=0.1, stamp=marker_stamp))
                    current_id += 1
                    marker_array.markers.append(self.create_points_marker(f'hand_{side}_tips', current_id, hand_lms, FINGERTIP_INDICES, (1.0, 1.0, 0.0), hand_transform, point_size=0.01, lifetime_sec=0.1, stamp=marker_stamp))
                    current_id += 1

                    wrist_pos = hand_transform(hand_lms[0])
                    hand_quat = self.compute_hand_quaternion(hand_lms, side)
                    hand_quat = self.smooth_quaternion(side, hand_quat)

                    t_hand = TransformStamped()
                    # TF uses tf_stamp (Valid time, usually from image header)
                    t_hand.header.stamp = tf_stamp 
                    t_hand.header.frame_id = 'landmark'
                    t_hand.child_frame_id = f'{side}_hand'
                    t_hand.transform.translation.x = wrist_pos.x
                    t_hand.transform.translation.y = wrist_pos.y
                    t_hand.transform.translation.z = wrist_pos.z
                    t_hand.transform.rotation = hand_quat
                    self.tf_broadcaster.sendTransform(t_hand)
        else:
            self.prev_pose_world = [None] * 33
            self.prev_hand_wrist_pos = {'left': None, 'right': None}
            self.prev_quats = {'left': None, 'right': None}
            
            for ns_suffix in ['body_edges', 'body_joints', 'hand_left_edges', 'hand_left_joints', 'hand_left_tips', 
                              'hand_right_edges', 'hand_right_joints', 'hand_right_tips']:
                del_marker = Marker()
                del_marker.header.frame_id = 'landmark'
                del_marker.header.stamp = marker_stamp # Time 0
                del_marker.ns = ns_suffix
                del_marker.id = 0
                del_marker.action = Marker.DELETEALL
                marker_array.markers.append(del_marker)
                
            for side in ['left', 'right']:
                t = TransformStamped()
                t.header.stamp = tf_stamp # Valid time
                t.header.frame_id = 'landmark'
                t.child_frame_id = f'{side}_hand'
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

        if not marker_array.markers:
            clear = Marker()
            clear.header.frame_id = 'landmark'
            clear.header.stamp = marker_stamp # Time 0
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)

        self.landmark_pub.publish(marker_array)
        
    def destroy_node(self):
        self.mp_pose.close(); self.mp_hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeBodyNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()