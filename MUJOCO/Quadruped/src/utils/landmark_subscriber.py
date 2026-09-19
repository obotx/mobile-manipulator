import threading
from typing import List, Tuple
import babyros

class LandmarkSubscriber:
    def __init__(self, max_keypoints: int = 133):
        self.max_keypoints = max_keypoints

        # Initialize with invalid zero poses
        self._poses = [{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "valid": False} for _ in range(self.max_keypoints)]
        self._poses_lock = threading.Lock()

        self._left_gesture = "unknown"
        self._right_gesture = "unknown"
        self._gesture_lock = threading.Lock()

        self._landmark_sub = babyros.node.Subscriber(
            topic="landmarks", callback=self._landmark_callback
        )
        self._gesture_sub = babyros.node.Subscriber(
            topic="hand_gestures", callback=self._gesture_callback
        )
        print("[LandmarkSubscriber] Initialized")

    def _landmark_callback(self, msg: dict):
        with self._poses_lock:
            # Reset all to invalid
            self._poses = [{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "valid": False} for _ in range(self.max_keypoints)]

            # Only mark points that are actually in the message as valid
            for part_name in ("body", "left_hand", "right_hand"):
                part_data = msg.get(part_name, [])
                for p in part_data:
                    kp_id = p.get("keypoint_id", -1)
                    if 0 <= kp_id < self.max_keypoints:
                        p["valid"] = True  # Explicitly mark as published
                        self._poses[kp_id] = p

    def _gesture_callback(self, msg: dict):
        with self._gesture_lock:
            self._left_gesture = msg.get("left_hand", "unknown")
            self._right_gesture = msg.get("right_hand", "unknown")

    def get_poses(self) -> List[dict]:
        with self._poses_lock:
            return self._poses.copy()

    def get_gestures(self) -> Tuple[str, str]:
        with self._gesture_lock:
            return self._left_gesture, self._right_gesture

    def cleanup(self):
        self._landmark_sub.delete()
        self._gesture_sub.delete()
        print("[LandmarkSubscriber] Cleaned up")
