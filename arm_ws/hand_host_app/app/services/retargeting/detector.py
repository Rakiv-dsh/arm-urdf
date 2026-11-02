"""
手部关键点检测器 - 基于MediaPipe
"""
import numpy as np
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2

# 坐标系转换矩阵
OPERATOR2MANO_RIGHT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
OPERATOR2MANO_LEFT = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])


def _require_mediapipe_solutions() -> None:
    if hasattr(mp, "solutions"):
        return
    version = getattr(mp, "__version__", "unknown")
    raise RuntimeError(
        f"当前 mediapipe {version} 不包含旧版 mp.solutions API；"
        "请安装兼容版本: python3 -m pip install --force-reinstall 'mediapipe<0.10.22'"
    )


class SingleHandDetector:
    """单手检测器"""

    def __init__(self, hand_type="Right", min_detection_confidence=0.8,
                 min_tracking_confidence=0.8, selfie=False):
        _require_mediapipe_solutions()
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.operator2mano = OPERATOR2MANO_RIGHT if hand_type == "Right" else OPERATOR2MANO_LEFT
        self.detected_hand_type = hand_type

    def detect(self, rgb):
        """检测手部关键点"""
        results = self.hand_detector.process(rgb)
        if not results.multi_hand_landmarks:
            return 0, None, None, None

        # 查找目标手
        desired_hand_num = -1
        for idx, handedness in enumerate(results.multi_handedness):
            label = handedness.classification[0].label
            if label == self.detected_hand_type:
                desired_hand_num = idx
                break

        if desired_hand_num < 0:
            return 0, None, None, None

        keypoint_3d = results.multi_hand_world_landmarks[desired_hand_num]
        keypoint_2d = results.multi_hand_landmarks[desired_hand_num]
        num_box = len(results.multi_hand_landmarks)

        # 解析3D关键点
        keypoint_3d_array = self._parse_keypoint_3d(keypoint_3d)
        keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]

        # 估计手部坐标系
        mediapipe_wrist_rot = self._estimate_frame(keypoint_3d_array)
        joint_pos = keypoint_3d_array @ mediapipe_wrist_rot @ self.operator2mano

        return num_box, joint_pos, keypoint_2d, mediapipe_wrist_rot

    @staticmethod
    def _parse_keypoint_3d(keypoint_3d) -> np.ndarray:
        """解析3D关键点"""
        keypoint = np.empty([21, 3])
        for i in range(21):
            keypoint[i][0] = keypoint_3d.landmark[i].x
            keypoint[i][1] = keypoint_3d.landmark[i].y
            keypoint[i][2] = keypoint_3d.landmark[i].z
        return keypoint

    @staticmethod
    def _estimate_frame(keypoint_3d_array: np.ndarray) -> np.ndarray:
        """估计手部坐标系"""
        assert keypoint_3d_array.shape == (21, 3)
        points = keypoint_3d_array[[0, 5, 9], :]
        x_vector = points[0] - points[2]
        points = points - np.mean(points, axis=0, keepdims=True)
        u, s, v = np.linalg.svd(points)
        normal = v[2, :]
        x = x_vector - np.sum(x_vector * normal) * normal
        x = x / np.linalg.norm(x)
        z = np.cross(x, normal)
        if np.sum(z * (points[1] - points[2])) < 0:
            normal *= -1
            z *= -1
        frame = np.stack([x, normal, z], axis=1)
        return frame

    @staticmethod
    def draw_skeleton_on_image(image, keypoint_2d, style="default"):
        """在图像上绘制骨架"""
        if keypoint_2d is None:
            return image

        if style == "default":
            mp.solutions.drawing_utils.draw_landmarks(
                image,
                keypoint_2d,
                mp.solutions.hands.HAND_CONNECTIONS,
                mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                mp.solutions.drawing_styles.get_default_hand_connections_style(),
            )
        return image
