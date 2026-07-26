import cv2
import mediapipe as mp
import numpy as np
import os
import json
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge  # <-- ADDED

from sensor_msgs.msg import Image, CompressedImage  # <-- ADDED Image
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose


class GestureRecognizer:
    """Recognize hand gestures (open/close) from MediaPipe landmarks."""
    THUMB_TIP = 4; THUMB_IP = 3; THUMB_MCP = 2; THUMB_CMC = 1
    INDEX_TIP = 8; INDEX_PIP = 6; INDEX_MCP = 5
    MIDDLE_TIP = 12; MIDDLE_PIP = 10; MIDDLE_MCP = 9
    RING_TIP = 16; RING_PIP = 14; RING_MCP = 13
    PINKY_TIP = 20; PINKY_PIP = 18; PINKY_MCP = 17
    WRIST = 0
    
    def __init__(self, open_ratio=0.6):
        self.open_ratio = open_ratio
    
    def is_finger_extended(self, landmarks, tip_idx, pip_idx, mcp_idx=None):
        tip = np.array([landmarks[tip_idx].x, landmarks[tip_idx].y, landmarks[tip_idx].z])
        pip = np.array([landmarks[pip_idx].x, landmarks[pip_idx].y, landmarks[pip_idx].z])
        wrist = np.array([landmarks[self.WRIST].x, landmarks[self.WRIST].y, landmarks[self.WRIST].z])
        
        if mcp_idx is not None:
            mcp = np.array([landmarks[mcp_idx].x, landmarks[mcp_idx].y, landmarks[mcp_idx].z])
            tip_to_pip = np.linalg.norm(tip - pip)
            mcp_to_pip = np.linalg.norm(mcp - pip)
            return tip_to_pip > mcp_to_pip * 1.2
        else:
            tip_to_wrist = np.linalg.norm(tip - wrist)
            pip_to_wrist = np.linalg.norm(pip - wrist)
            return tip_to_wrist > pip_to_wrist + 0.02
    
    def recognize_gesture(self, landmarks):
        if len(landmarks) < 21:
            return "unknown"
        
        fingers_extended = []
        fingers_extended.append(self.is_finger_extended(landmarks, self.THUMB_TIP, self.THUMB_IP, self.INDEX_MCP))
        fingers_extended.append(self.is_finger_extended(landmarks, self.INDEX_TIP, self.INDEX_PIP))
        fingers_extended.append(self.is_finger_extended(landmarks, self.MIDDLE_TIP, self.MIDDLE_PIP))
        fingers_extended.append(self.is_finger_extended(landmarks, self.RING_TIP, self.RING_PIP))
        fingers_extended.append(self.is_finger_extended(landmarks, self.PINKY_TIP, self.PINKY_PIP))
        
        extended_count = sum(fingers_extended)
        total_fingers = len(fingers_extended)
        
        if extended_count >= total_fingers * self.open_ratio:
            return "open"
        elif extended_count <= total_fingers * (1 - self.open_ratio):
            return "close"
        else:
            return "partial"


class BodyPreFocus:    
    def __init__(self, hand_padding_factor=1.5, min_hand_size=100):
        self.hand_padding_factor = hand_padding_factor
        self.min_hand_size = min_hand_size
        
    def get_wrist_positions(self, pose_landmarks, frame_width, frame_height):
        if pose_landmarks is None or len(pose_landmarks.landmark) < 16:
            return None, None
        
        left_wrist = pose_landmarks.landmark[15]
        right_wrist = pose_landmarks.landmark[16]
        left_elbow = pose_landmarks.landmark[13]
        right_elbow = pose_landmarks.landmark[14]
        
        left_wrist_px = (int(left_wrist.x * frame_width), int(left_wrist.y * frame_height))
        right_wrist_px = (int(right_wrist.x * frame_width), int(right_wrist.y * frame_height))
        left_elbow_px = (int(left_elbow.x * frame_width), int(left_elbow.y * frame_height))
        right_elbow_px = (int(right_elbow.x * frame_width), int(right_elbow.y * frame_height))
        
        return (left_wrist_px, left_elbow_px), (right_wrist_px, right_elbow_px)
    
    def create_hand_crop(self, wrist_px, elbow_px, frame_width, frame_height):
        wrist_x, wrist_y = wrist_px
        elbow_x, elbow_y = elbow_px
        
        arm_length = np.sqrt((wrist_x - elbow_x)**2 + (wrist_y - elbow_y)**2)
        hand_size = int(arm_length * self.hand_padding_factor)
        hand_size = max(hand_size, self.min_hand_size)
        
        x_min = max(0, wrist_x - hand_size)
        y_min = max(0, wrist_y - hand_size)
        x_max = min(frame_width, wrist_x + hand_size)
        y_max = min(frame_height, wrist_y + hand_size)
        
        if x_max - x_min < self.min_hand_size:
            center_x = (x_min + x_max) // 2
            x_min = max(0, center_x - self.min_hand_size // 2)
            x_max = min(frame_width, center_x + self.min_hand_size // 2)
            
        if y_max - y_min < self.min_hand_size:
            center_y = (y_min + y_max) // 2
            y_min = max(0, center_y - self.min_hand_size // 2)
            y_max = min(frame_height, center_y + self.min_hand_size // 2)
        
        return (x_min, y_min, x_max, y_max)
    
    def shift_landmarks_to_original(self, landmarks, crop_box, frame_width, frame_height):
        x_min, y_min, _, _ = crop_box
        for landmark in landmarks.landmark:
            landmark.x = (landmark.x * (crop_box[2] - crop_box[0]) + x_min) / frame_width
            landmark.y = (landmark.y * (crop_box[3] - crop_box[1]) + y_min) / frame_height


class EMAFilter:
    def __init__(self, alpha=0.4):
        self.alpha = alpha
        self.prev_points = None

    def filter(self, points):
        if self.prev_points is None:
            self.prev_points = points.copy()
            return points
        filtered = self.alpha * points + (1 - self.alpha) * self.prev_points
        self.prev_points = filtered
        return filtered


class RigidTransformFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.prev_rvec = None
        self.prev_tvec = None

    def filter(self, rvec, tvec):
        if self.prev_rvec is None:
            self.prev_rvec = rvec.copy()
            self.prev_tvec = tvec.copy()
            return rvec, tvec
        
        filtered_rvec = self.alpha * rvec + (1 - self.alpha) * self.prev_rvec
        filtered_tvec = self.alpha * tvec + (1 - self.alpha) * self.prev_tvec
        
        self.prev_rvec = filtered_rvec
        self.prev_tvec = filtered_tvec
        return filtered_rvec, filtered_tvec


def get_camera_matrix(frame_width, frame_height):
    focal_length = frame_width
    center = (frame_width / 2, frame_height / 2)
    camera_matrix = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype="double")
    distortion = np.zeros((4, 1))
    return camera_matrix, distortion


def load_calibration(calib_file, frame_width, frame_height):
    if calib_file and os.path.exists(calib_file):
        try:
            calib_data = np.load(calib_file)
            return calib_data["camera_matrix"], calib_data["dist_coeffs"]
        except Exception as e:
            print(f" Error loading calibration: {e}. Falling back.")
    return get_camera_matrix(frame_width, frame_height)


class SyntheticResults:
    def __init__(self, landmarks_list, sides_list):
        self.multi_hand_landmarks = landmarks_list
        self.multi_handedness = None
        self.multi_hand_world_landmarks = None
        self.hand_sides = sides_list


class HandPoseTrackerNode(Node):
    def __init__(self):
        super().__init__('hand_pose_tracker_node')
        
        # Declare parameters
        self.declare_parameter('calibration_file', 'cam_calib_(10x7)_22.0mm.npz')
        self.declare_parameter('smoothing', 0.10)
        self.declare_parameter('detect_conf_hand', 0.3)
        self.declare_parameter('track_conf_hand', 0.3)
        self.declare_parameter('detect_conf_pose', 0.5)
        self.declare_parameter('track_conf_pose', 0.5)
        self.declare_parameter('hand_padding', 3.0)
        self.declare_parameter('use_bpf', False)
        self.declare_parameter('enhance', False)
        self.declare_parameter('publish_2d', True)

        # Get parameters
        self.calib_file = self.get_parameter('calibration_file').value
        self.smoothing = self.get_parameter('smoothing').value
        self.use_bpf = self.get_parameter('use_bpf').value
        self.enhance = self.get_parameter('enhance').value
        self.publish_2d = self.get_parameter('publish_2d').value

        # Publishers
        self.pub_tracking = self.create_publisher(PoseArray, '/landmarks', 10)
        self.pub_gesture = self.create_publisher(String, '/hand_gestures', 10)
        self.pub_image = self.create_publisher(CompressedImage, '/image_landmark', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        self.get_logger().info("Publishers initialized: '/landmarks', '/hand_gestures', '/image_landmark', '/visualization_marker_array'")
        if self.use_bpf:
            self.get_logger().info(f"Body Pre-Focusing ENABLED (hand_padding={self.get_parameter('hand_padding').value})")
        if not self.publish_2d:
            self.get_logger().info("2D Image Publishing DISABLED (Headless mode)")

        # Components
        self.bpf = BodyPreFocus(hand_padding_factor=self.get_parameter('hand_padding').value) if self.use_bpf else None
        self.gesture_recognizer = GestureRecognizer()
        
        if self.enhance:
            self.get_logger().info("Software Image Enhancement ENABLED (CLAHE + Unsharp Masking)")
            self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        else:
            self.clahe = None

        # Filters
        self.hand_rigid_filter_left = RigidTransformFilter(alpha=self.smoothing)
        self.hand_rigid_filter_right = RigidTransformFilter(alpha=self.smoothing)
        self.pose_filter = EMAFilter(alpha=self.smoothing)
        self.pose_rigid_filter = RigidTransformFilter(alpha=self.smoothing)

        # MediaPipe Models
        self.hands = mp_hands.Hands(
            model_complexity=0, max_num_hands=2,
            min_detection_confidence=self.get_parameter('detect_conf_hand').value,
            min_tracking_confidence=self.get_parameter('track_conf_hand').value
        )
        self.pose = mp_pose.Pose(
            model_complexity=2, smooth_landmarks=True, enable_segmentation=True,
            min_detection_confidence=self.get_parameter('detect_conf_pose').value,
            min_tracking_confidence=self.get_parameter('track_conf_pose').value
        )

        # CvBridge for converting ROS Image to OpenCV
        self.bridge = CvBridge()

        # Subscriber: CHANGED to Image and /image_raw
        self.sub_image = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            1
        )
        
        self.get_logger().info("Starting Tracking & Publishing with Gesture Recognition...")
        self.frame_count = 0

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV BGR format
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion error: {e}")
            return

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        frame_h, frame_w = image.shape[:2]

        # Load calibration on first frame
        if not hasattr(self, 'camera_matrix'):
            self.camera_matrix, self.distortion = load_calibration(self.calib_file, frame_w, frame_h)

        # Enhance
        if self.clahe is not None:
            lab = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2LAB)
            l, a, b = cv2.split(lab)
            l = self.clahe.apply(l)
            lab = cv2.merge((l, a, b))
            image_rgb = cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)
            blurred = cv2.GaussianBlur(image_rgb, (0, 0), 2.0)
            image_rgb = cv2.addWeighted(image_rgb, 1.5, blurred, -0.5, 0)

        # Pose Processing
        results_pose = self.pose.process(image_rgb)
        left_wrist_elbow, right_wrist_elbow = None, None
        left_crop, right_crop = None, None
        
        if results_pose.pose_landmarks and self.bpf is not None and self.use_bpf:
            left_wrist_elbow, right_wrist_elbow = self.bpf.get_wrist_positions(
                results_pose.pose_landmarks, frame_w, frame_h
            )
            if left_wrist_elbow:
                left_crop = self.bpf.create_hand_crop(left_wrist_elbow[0], left_wrist_elbow[1], frame_w, frame_h)
            if right_wrist_elbow:
                right_crop = self.bpf.create_hand_crop(right_wrist_elbow[0], right_wrist_elbow[1], frame_w, frame_h)

        # Hand Processing (with BPF logic)
        results_hands = None
        hands_detected_in_crop = False
        
        if self.use_bpf and self.bpf is not None and (left_crop or right_crop):
            if left_crop and right_crop:
                x_min = min(left_crop[0], right_crop[0])
                y_min = min(left_crop[1], right_crop[1])
                x_max = max(left_crop[2], right_crop[2])
                y_max = max(left_crop[3], right_crop[3])
                combined_crop = (x_min, y_min, x_max, y_max)
                
                if x_max > x_min and y_max > y_min:
                    cropped_image = image_rgb[y_min:y_max, x_min:x_max]
                    if cropped_image.size > 0:
                        results_hands = self.hands.process(cropped_image)
                        if results_hands.multi_hand_landmarks:
                            hands_detected_in_crop = True
                            for hand_landmarks in results_hands.multi_hand_landmarks:
                                self.bpf.shift_landmarks_to_original(hand_landmarks, combined_crop, frame_w, frame_h)
            else:
                crops_to_process = []
                if left_crop: crops_to_process.append(('left', left_crop))
                if right_crop: crops_to_process.append(('right', right_crop))
                
                all_hand_landmarks = []
                all_hand_sides = []
                
                for side, crop in crops_to_process:
                    x_min, y_min, x_max, y_max = crop
                    if x_max <= x_min or y_max <= y_min: continue
                    cropped_image = image_rgb[y_min:y_max, x_min:x_max]
                    if cropped_image.size == 0: continue
                        
                    crop_results = self.hands.process(cropped_image)
                    if crop_results.multi_hand_landmarks:
                        hands_detected_in_crop = True
                        for hand_landmarks in crop_results.multi_hand_landmarks:
                            self.bpf.shift_landmarks_to_original(hand_landmarks, crop, frame_w, frame_h)
                            all_hand_landmarks.append(hand_landmarks)
                            all_hand_sides.append(side)
                
                if all_hand_landmarks:
                    results_hands = SyntheticResults(all_hand_landmarks, all_hand_sides)

            if not hands_detected_in_crop:
                results_hands = self.hands.process(image_rgb)
        else:
            results_hands = self.hands.process(image_rgb)

        # 3D Pose Calculation
        pose_world_points = None
        if results_pose.pose_world_landmarks and results_pose.pose_landmarks:
            model_points = np.array([[lm.x, lm.y, lm.z] for lm in results_pose.pose_world_landmarks.landmark])
            image_points = np.array([[lm.x * frame_w, lm.y * frame_h] for lm in results_pose.pose_landmarks.landmark])

            success_pnp, rvec, tvec = cv2.solvePnP(model_points, image_points, self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_SQPNP)
            if success_pnp:
                rvec, tvec = self.pose_rigid_filter.filter(rvec, tvec)
                rmat, _ = cv2.Rodrigues(rvec)
                transformation = np.eye(4)
                transformation[0:3, 0:3] = rmat
                transformation[0:3, 3] = tvec.squeeze()

                model_points_hom = np.concatenate((model_points, np.ones((33, 1))), axis=1)
                pose_world_points = model_points_hom.dot(transformation.T)[:, :3]
                pose_world_points[:, 1] = -pose_world_points[:, 1]
                pose_world_points[:, 2] = -pose_world_points[:, 2]
                pose_world_points = self.pose_filter.filter(pose_world_points)

        # 3D Hand Calculation & Gestures
        left_hand_final_points = None
        right_hand_final_points = None
        left_gesture = "unknown"
        right_gesture = "unknown"

        if results_hands and getattr(results_hands, 'multi_hand_landmarks', None):
            for idx, hand_landmarks in enumerate(results_hands.multi_hand_landmarks):
                gesture = self.gesture_recognizer.recognize_gesture(hand_landmarks.landmark)
                
                if results_hands.multi_hand_world_landmarks and idx < len(results_hands.multi_hand_world_landmarks):
                    world_landmarks = results_hands.multi_hand_world_landmarks[idx]
                    hand_model_points = np.array([[lm.x, lm.y, lm.z] for lm in world_landmarks.landmark])
                else:
                    hand_model_points = np.array([[lm.x, lm.y, lm.z] for lm in hand_landmarks.landmark])
                
                hand_image_points = np.array([[lm.x * frame_w, lm.y * frame_h] for lm in hand_landmarks.landmark])
                
                if hasattr(results_hands, 'hand_sides') and idx < len(results_hands.hand_sides):
                    is_left_hand = (results_hands.hand_sides[idx] == 'left')
                else:
                    wrist_x = hand_image_points[0, 0]
                    is_left_hand = (wrist_x > frame_w / 2)
                
                if is_left_hand: left_gesture = gesture
                else: right_gesture = gesture
                
                success_pnp, rvec, tvec = cv2.solvePnP(hand_model_points, hand_image_points, self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_SQPNP)
                if success_pnp:
                    if is_left_hand:
                        rvec, tvec = self.hand_rigid_filter_left.filter(rvec, tvec)
                    else:
                        rvec, tvec = self.hand_rigid_filter_right.filter(rvec, tvec)

                    rmat, _ = cv2.Rodrigues(rvec)
                    transformation = np.eye(4)
                    transformation[0:3, 0:3] = rmat
                    transformation[0:3, 3] = tvec.squeeze()
                    
                    hand_model_hom = np.concatenate((hand_model_points, np.ones((21, 1))), axis=1)
                    hand_world_points = hand_model_hom.dot(transformation.T)[:, :3]
                    hand_world_points[:, 1] = -hand_world_points[:, 1]
                    hand_world_points[:, 2] = -hand_world_points[:, 2]
                    
                    if pose_world_points is not None:
                        if is_left_hand and len(pose_world_points) > 15:
                            offset = hand_world_points - hand_world_points[0]
                            hand_world_points = pose_world_points[15] + offset
                        elif not is_left_hand and len(pose_world_points) > 16:
                            offset = hand_world_points - hand_world_points[0]
                            hand_world_points = pose_world_points[16] + offset
                    
                    if is_left_hand:
                        left_hand_final_points = hand_world_points.copy()
                    else:
                        right_hand_final_points = hand_world_points.copy()

        # --- PUBLISHING ---
        current_time = self.get_clock().now().to_msg()

        # 1. PoseArray (Landmarks)
        poses_list = []
        if pose_world_points is not None and len(pose_world_points) == 33:
            for pt in pose_world_points:
                poses_list.append({"position": {"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}})
        if left_hand_final_points is not None and len(left_hand_final_points) == 21:
            for pt in left_hand_final_points:
                poses_list.append({"position": {"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}})
        if right_hand_final_points is not None and len(right_hand_final_points) == 21:
            for pt in right_hand_final_points:
                poses_list.append({"position": {"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}})

        if len(poses_list) > 0:
            pose_array_msg = PoseArray()
            pose_array_msg.header.stamp = current_time
            pose_array_msg.header.frame_id = "world"
            for p in poses_list:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = p["position"]["x"], p["position"]["y"], p["position"]["z"]
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = p["orientation"]["x"], p["orientation"]["y"], p["orientation"]["z"], p["orientation"]["w"]
                pose_array_msg.poses.append(pose)
            self.pub_tracking.publish(pose_array_msg)

        # 2. Gestures
        gesture_msg = String()
        gesture_msg.data = json.dumps({"left_hand": left_gesture, "right_hand": right_gesture})
        self.pub_gesture.publish(gesture_msg)

        # 3. MarkerArray (3D Visualization for RViz)
        marker_array = MarkerArray()
        stamp = current_time
        frame_id = "world"

        def add_sphere_list(points, ns, id, color, scale=0.02):
            marker = Marker()
            marker.header.stamp = stamp; marker.header.frame_id = frame_id
            marker.ns = ns; marker.id = id
            marker.type = Marker.SPHERE_LIST; marker.action = Marker.ADD
            marker.scale.x = scale; marker.scale.y = scale; marker.scale.z = scale
            marker.color = color
            for pt in points:
                p = Point(); p.x, p.y, p.z = float(pt[0]), float(pt[1]), float(pt[2])
                marker.points.append(p)
            marker_array.markers.append(marker)

        def add_line_list(points, connections, ns, id, color, scale=0.005):
            marker = Marker()
            marker.header.stamp = stamp; marker.header.frame_id = frame_id
            marker.ns = ns; marker.id = id
            marker.type = Marker.LINE_LIST; marker.action = Marker.ADD
            marker.scale.x = scale; marker.color = color
            for conn in connections:
                p1, p2 = points[conn[0]], points[conn[1]]
                pt1 = Point(); pt1.x, pt1.y, pt1.z = float(p1[0]), float(p1[1]), float(p1[2])
                pt2 = Point(); pt2.x, pt2.y, pt2.z = float(p2[0]), float(p2[1]), float(p2[2])
                marker.points.append(pt1); marker.points.append(pt2)
            marker_array.markers.append(marker)

        if pose_world_points is not None:
            add_sphere_list(pose_world_points, "pose_points", 0, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), 0.03)
            add_line_list(pose_world_points, mp_pose.POSE_CONNECTIONS, "pose_lines", 1, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), 0.008)
        if left_hand_final_points is not None:
            add_sphere_list(left_hand_final_points, "left_hand_points", 2, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), 0.02)
            add_line_list(left_hand_final_points, mp_hands.HAND_CONNECTIONS, "left_hand_lines", 3, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), 0.005)
        if right_hand_final_points is not None:
            add_sphere_list(right_hand_final_points, "right_hand_points", 4, ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0), 0.02)
            add_line_list(right_hand_final_points, mp_hands.HAND_CONNECTIONS, "right_hand_lines", 5, ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0), 0.005)

        if len(marker_array.markers) > 0:
            self.pub_markers.publish(marker_array)

        # 4. 2D Annotated Image Publishing (Conditional)
        if self.publish_2d:
            image_bgr = image # Already in BGR from CvBridge
            
            # Draw landmarks
            if results_pose.pose_landmarks:
                mp_drawing.draw_landmarks(
                    image_bgr, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    mp_drawing_styles.get_default_pose_landmarks_style())
            
            if results_hands and getattr(results_hands, 'multi_hand_landmarks', None):
                for hand_landmarks in results_hands.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image_bgr, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                
                if self.use_bpf and self.bpf is not None:
                    if left_crop:
                        x_min, y_min, x_max, y_max = left_crop
                        cv2.rectangle(image_bgr, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                        cv2.putText(image_bgr, "Left Hand ROI", (x_min, y_min-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    if right_crop:
                        x_min, y_min, x_max, y_max = right_crop
                        cv2.rectangle(image_bgr, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
                        cv2.putText(image_bgr, "Right Hand ROI", (x_min, y_min-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Add text overlays
            cv2.putText(image_bgr, f"BPF: {'ON' if self.use_bpf else 'OFF'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if self.use_bpf else (0, 0, 255), 2)
            cv2.putText(image_bgr, f"Enhance: {'ON' if self.clahe is not None else 'OFF'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if self.clahe is not None else (0, 0, 255), 2)
            cv2.putText(image_bgr, f"Left: {left_gesture}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(image_bgr, f"Right: {right_gesture}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # Encode and publish
            _, encoded_image = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            comp_img_msg = CompressedImage()
            comp_img_msg.header = msg.header
            comp_img_msg.header.stamp = current_time
            comp_img_msg.format = "jpeg"
            comp_img_msg.data = encoded_image.tobytes()
            
            self.pub_image.publish(comp_img_msg)

        self.frame_count += 1

    def destroy_node(self):
        self.hands.close()
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandPoseTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()