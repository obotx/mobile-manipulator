#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import cv2
import numpy as np
import mediapipe as mp

HAND_CONNECTIONS = list(mp.solutions.hands.HAND_CONNECTIONS)
FINGERTIP_INDICES = [4, 8, 12, 16, 20]


class MediaPipeHandsNode(Node):

    def __init__(self):
        super().__init__('mediapipe_hands_node')

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.compressed_callback,
            qos_profile=qos_img
        )

        self.landmark_pub = self.create_publisher(
            MarkerArray,
            '/hand_landmarks',
            10
        )

        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("MediaPipe hands node started (direct world coordinates)")

    def create_line_marker(self, ns, marker_id, landmarks, connections, color, transform_fn, line_width=0.005, lifetime_sec=0.1):
        marker = Marker()
        marker.header.frame_id = 'cam'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = line_width
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.lifetime = Duration(seconds=lifetime_sec).to_msg()

        for start, end in connections:
            marker.points.append(transform_fn(landmarks[start]))
            marker.points.append(transform_fn(landmarks[end]))
        return marker

    def create_points_marker(self, ns, marker_id, landmarks, indices, color, transform_fn, point_size=0.015, lifetime_sec=0.1):
        marker = Marker()
        marker.header.frame_id = 'cam'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y = point_size, point_size
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.lifetime = Duration(seconds=lifetime_sec).to_msg()

        for idx in indices:
            marker.points.append(transform_fn(landmarks[idx]))
        return marker

    def compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                return
        except Exception as e:
            self.get_logger().error(f"Decode failed: {e}")
            return

        cv_image = cv2.resize(cv_image, (640, 480))
        cv_image = cv2.flip(cv_image, 1)  # Mirror for natural self-view
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        hand_results = self.mp_hands.process(rgb)

        marker_array = MarkerArray()
        current_id = 0

        # Direct pass-through: MediaPipe world (meters) -> ROS Point
        def direct_world_transform(lm):
            p = Point()
            p.x = lm.x
            p.y = lm.y
            p.z = lm.z
            return p

        if hand_results.multi_hand_world_landmarks:
            for hand_idx, hand_world in enumerate(hand_results.multi_hand_world_landmarks):
                hand_lms = hand_world.landmark

                # Skeleton lines
                marker_array.markers.append(
                    self.create_line_marker(
                        f'hand_{hand_idx}_edges', current_id, hand_lms, HAND_CONNECTIONS,
                        (1.0, 0.0, 0.0), transform_fn=direct_world_transform,
                        line_width=0.005, lifetime_sec=0.1
                    )
                )
                current_id += 1

                # All 21 joints
                marker_array.markers.append(
                    self.create_points_marker(
                        f'hand_{hand_idx}_joints', current_id, hand_lms, list(range(21)),
                        (1.0, 0.0, 0.0), transform_fn=direct_world_transform,
                        point_size=0.015, lifetime_sec=0.1
                    )
                )
                current_id += 1

                # Fingertips
                marker_array.markers.append(
                    self.create_points_marker(
                        f'hand_{hand_idx}_tips', current_id, hand_lms, FINGERTIP_INDICES,
                        (1.0, 1.0, 0.0), transform_fn=direct_world_transform,
                        point_size=0.03, lifetime_sec=0.1
                    )
                )
                current_id += 1
        else:
            # Clean up markers when hands disappear
            for i in range(2):
                for suffix in ['edges', 'joints', 'tips']:
                    del_marker = Marker()
                    del_marker.header.frame_id = 'cam'
                    del_marker.header.stamp = self.get_clock().now().to_msg()
                    del_marker.ns = f'hand_{i}_{suffix}'
                    del_marker.id = 0
                    del_marker.action = Marker.DELETEALL
                    marker_array.markers.append(del_marker)

        if not marker_array.markers:
            clear = Marker()
            clear.header.frame_id = 'cam'
            clear.header.stamp = self.get_clock().now().to_msg()
            clear.action = Marker.DELETEALL
            marker_array.markers.append(clear)

        self.landmark_pub.publish(marker_array)

    def destroy_node(self):
        self.mp_hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeHandsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()