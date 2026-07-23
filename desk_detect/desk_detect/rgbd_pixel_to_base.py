#!/usr/bin/env python3
"""
RGB-D 2D 픽셀 -> 카메라 3D 좌표 -> 로봇 Base 좌표 변환 모듈

이 파일은 YOLO, MediaPipe 등 특정 인식 모델에 의존하지 않는다.
인식 코드에서 얻은 객체 중심 픽셀 (u, v)만 전달하면 다음 과정을 수행한다.

    객체/손 검출
        -> 영상의 2D 중심 픽셀 (u, v)
        -> aligned Depth에서 깊이 Z 획득
        -> CameraInfo로 카메라 기준 3D 좌표 [Xc, Yc, Zc]
        -> Hand-Eye 행렬로 calibration_frame 기준 좌표 변환
        -> TF로 base_frame 기준 좌표 [Xb, Yb, Zb] 변환

중요:
- 검출기가 2D 좌표를 만든다.
- Depth 값 하나만으로 2D 좌표를 만드는 것이 아니다.
- 2D 픽셀 (u, v)와 해당 픽셀의 Depth Z를 결합하여 3D 좌표를 만든다.
- aligned_depth_to_color 영상을 사용해야 컬러 픽셀과 Depth 픽셀이 일치한다.

기본 프레임 구성:
    base_frame        = base_link
    calibration_frame = link_6

T_gripper2camera.npy가 link_6 기준으로 캘리브레이션되었다면
calibration_frame은 link_6로 설정한다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


@dataclass(frozen=True)
class CoordinateResult:
    """한 픽셀에 대한 좌표 변환 결과."""

    pixel: Tuple[int, int]
    depth_m: float
    camera_point_m: np.ndarray
    base_point_m: np.ndarray

    @property
    def base_point_mm(self) -> np.ndarray:
        return self.base_point_m * 1000.0


def bbox_center_xyxy(
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
) -> Tuple[int, int]:
    """YOLO xyxy 바운딩박스에서 중심 픽셀을 계산한다."""
    center_u = int(round((float(x_min) + float(x_max)) / 2.0))
    center_v = int(round((float(y_min) + float(y_max)) / 2.0))
    return center_u, center_v


def quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    """TF quaternion을 3x3 회전행렬로 변환한다."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)

    if norm < 1.0e-12:
        raise ValueError("Quaternion norm is zero.")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def load_camera_to_calibration_transform(
    transform_path: str,
    transform_direction: str = "gripper_to_camera",
    transform_translation_unit: str = "auto",
) -> np.ndarray:
    """
    Hand-Eye 행렬을 camera -> calibration_frame 형식으로 로드한다.

    반환:
        T_calibration_camera_m

        p_calibration = T_calibration_camera_m @ p_camera
    """
    path = Path(transform_path).expanduser().resolve()

    if not path.is_file():
        raise FileNotFoundError(f"Transform file not found: {path}")

    matrix = np.asarray(np.load(str(path)), dtype=np.float64)

    if matrix.shape == (3, 4):
        matrix = np.vstack(
            [
                matrix,
                np.array(
                    [[0.0, 0.0, 0.0, 1.0]],
                    dtype=np.float64,
                ),
            ]
        )

    if matrix.shape != (4, 4):
        raise ValueError(
            "Transform matrix must be 4x4 or 3x4. "
            f"Current shape: {matrix.shape}"
        )

    if not np.all(np.isfinite(matrix)):
        raise ValueError("Transform matrix contains NaN or Inf.")

    if abs(float(matrix[3, 3])) < 1.0e-12:
        raise ValueError("Invalid homogeneous transform matrix.")

    matrix = matrix / float(matrix[3, 3])

    direction = transform_direction.strip().lower()

    if direction == "gripper_to_camera":
        matrix = np.linalg.inv(matrix)
    elif direction != "camera_to_gripper":
        raise ValueError(
            "transform_direction must be "
            "'gripper_to_camera' or 'camera_to_gripper'."
        )

    unit = transform_translation_unit.strip().lower()
    translation_norm = float(np.linalg.norm(matrix[:3, 3]))

    if unit == "auto":
        detected_unit = "m" if translation_norm < 2.0 else "mm"
    elif unit in ("m", "mm"):
        detected_unit = unit
    else:
        raise ValueError(
            "transform_translation_unit must be 'auto', 'm', or 'mm'."
        )

    # 내부 단위를 ROS TF와 같은 meter로 통일한다.
    if detected_unit == "mm":
        matrix[:3, 3] /= 1000.0

    return matrix


def median_depth_at_pixel(
    depth_image: np.ndarray,
    depth_encoding: str,
    u: int,
    v: int,
    *,
    depth_scale_16u: float = 0.001,
    roi_radius: int = 5,
    min_valid_depth_m: float = 0.15,
    max_valid_depth_m: float = 2.0,
    min_valid_pixel_count: int = 5,
) -> Optional[float]:
    """
    2D 픽셀 주변 ROI에서 유효한 Depth의 중앙값을 반환한다.

    16UC1 Depth는 일반적으로 mm이므로 0.001을 곱해 m로 바꾼다.
    """
    if depth_image is None or depth_image.size == 0:
        return None

    height, width = depth_image.shape[:2]

    if not (0 <= u < width and 0 <= v < height):
        return None

    radius = max(0, int(roi_radius))

    roi = np.asarray(
        depth_image[
            max(0, v - radius):min(height, v + radius + 1),
            max(0, u - radius):min(width, u + radius + 1),
        ]
    )

    if roi.size == 0:
        return None

    values_m = roi.astype(np.float64)
    encoding = str(depth_encoding).lower()

    if roi.dtype == np.uint16 or encoding in ("16uc1", "mono16"):
        values_m *= float(depth_scale_16u)

    valid_values = values_m[
        np.isfinite(values_m)
        & (values_m >= float(min_valid_depth_m))
        & (values_m <= float(max_valid_depth_m))
    ]

    if valid_values.size < int(min_valid_pixel_count):
        return None

    return float(np.median(valid_values))


def deproject_pixel_to_camera(
    u: int,
    v: int,
    depth_m: float,
    camera_info: CameraInfo,
) -> np.ndarray:
    """
    픽셀 (u, v)와 Depth를 카메라 기준 3D 좌표로 변환한다.

    Xc = (u - cx) * Zc / fx
    Yc = (v - cy) * Zc / fy
    Zc = depth

    반환 단위: meter
    """
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])

    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo fx/fy must be greater than zero.")

    return np.array(
        [
            (float(u) - cx) * float(depth_m) / fx,
            (float(v) - cy) * float(depth_m) / fy,
            float(depth_m),
        ],
        dtype=np.float64,
    )


def lookup_base_to_frame_matrix(
    tf_buffer: tf2_ros.Buffer,
    base_frame: str,
    child_frame: str,
    timeout_sec: float = 0.05,
) -> Optional[np.ndarray]:
    """
    TF에서 base_frame <- child_frame 변환을 4x4 행렬로 반환한다.

    p_base = T_base_child @ p_child
    """
    try:
        transform = tf_buffer.lookup_transform(
            base_frame,
            child_frame,
            Time(),
            timeout=Duration(seconds=float(timeout_sec)),
        )
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ):
        return None

    translation = transform.transform.translation
    quaternion = transform.transform.rotation

    rotation = quaternion_to_rotation_matrix(
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [
        float(translation.x),
        float(translation.y),
        float(translation.z),
    ]

    return matrix


def transform_camera_point_to_base(
    camera_point_m: np.ndarray,
    transform_camera_to_calibration_m: np.ndarray,
    transform_base_to_calibration_m: np.ndarray,
) -> np.ndarray:
    """
    카메라 3D 좌표를 로봇 base_frame 좌표로 변환한다.

    p_calibration = T_calibration_camera @ p_camera
    p_base        = T_base_calibration @ p_calibration

    즉:
    p_base = T_base_calibration @ T_calibration_camera @ p_camera
    """
    point = np.asarray(camera_point_m, dtype=np.float64).reshape(3)

    point_camera_h = np.array(
        [point[0], point[1], point[2], 1.0],
        dtype=np.float64,
    )

    point_base_h = (
        transform_base_to_calibration_m
        @ transform_camera_to_calibration_m
        @ point_camera_h
    )

    w = float(point_base_h[3])

    if abs(w) < 1.0e-12:
        raise ValueError("Transformed homogeneous point has w=0.")

    point_base_m = point_base_h[:3] / w

    if not np.all(np.isfinite(point_base_m)):
        raise ValueError("Base point contains NaN or Inf.")

    return point_base_m


class RgbdPixelToBase:
    """
    다른 ROS2 인식 노드에서 재사용할 좌표 변환 클래스.

    입력:
        2D 픽셀 (u, v)
        aligned depth image
        depth encoding
        CameraInfo

    출력:
        카메라 기준 3D 좌표
        base_frame 기준 3D 좌표
    """

    def __init__(
        self,
        *,
        tf_buffer: tf2_ros.Buffer,
        transform_path: str,
        base_frame: str = "base_link",
        calibration_frame: str = "link_6",
        transform_direction: str = "gripper_to_camera",
        transform_translation_unit: str = "auto",
        tf_timeout_sec: float = 0.05,
        depth_scale_16u: float = 0.001,
        depth_roi_radius: int = 5,
        min_valid_depth_m: float = 0.15,
        max_valid_depth_m: float = 2.0,
    ) -> None:
        self.tf_buffer = tf_buffer
        self.base_frame = str(base_frame)
        self.calibration_frame = str(calibration_frame)
        self.tf_timeout_sec = float(tf_timeout_sec)

        self.depth_scale_16u = float(depth_scale_16u)
        self.depth_roi_radius = int(depth_roi_radius)
        self.min_valid_depth_m = float(min_valid_depth_m)
        self.max_valid_depth_m = float(max_valid_depth_m)

        self.T_calibration_camera_m = (
            load_camera_to_calibration_transform(
                transform_path=transform_path,
                transform_direction=transform_direction,
                transform_translation_unit=transform_translation_unit,
            )
        )

    def pixel_to_base(
        self,
        *,
        u: int,
        v: int,
        depth_image: np.ndarray,
        depth_encoding: str,
        camera_info: CameraInfo,
    ) -> Optional[CoordinateResult]:
        """검출된 중심 픽셀 하나를 base_frame 좌표로 변환한다."""
        depth_m = median_depth_at_pixel(
            depth_image=depth_image,
            depth_encoding=depth_encoding,
            u=int(u),
            v=int(v),
            depth_scale_16u=self.depth_scale_16u,
            roi_radius=self.depth_roi_radius,
            min_valid_depth_m=self.min_valid_depth_m,
            max_valid_depth_m=self.max_valid_depth_m,
        )

        if depth_m is None:
            return None

        camera_point_m = deproject_pixel_to_camera(
            u=int(u),
            v=int(v),
            depth_m=depth_m,
            camera_info=camera_info,
        )

        T_base_calibration_m = lookup_base_to_frame_matrix(
            tf_buffer=self.tf_buffer,
            base_frame=self.base_frame,
            child_frame=self.calibration_frame,
            timeout_sec=self.tf_timeout_sec,
        )

        if T_base_calibration_m is None:
            return None

        base_point_m = transform_camera_point_to_base(
            camera_point_m=camera_point_m,
            transform_camera_to_calibration_m=(
                self.T_calibration_camera_m
            ),
            transform_base_to_calibration_m=T_base_calibration_m,
        )

        return CoordinateResult(
            pixel=(int(u), int(v)),
            depth_m=depth_m,
            camera_point_m=camera_point_m,
            base_point_m=base_point_m,
        )


# ----------------------------------------------------------------------
# 다른 ROS2 코드에 붙이는 예시
# ----------------------------------------------------------------------
INTEGRATION_EXAMPLE = r"""
# Node.__init__에서 한 번 생성
from pathlib import Path
import tf2_ros
from rclpy.duration import Duration
from rgbd_pixel_to_base import RgbdPixelToBase, bbox_center_xyxy

self.tf_buffer = tf2_ros.Buffer(
    cache_time=Duration(seconds=10.0)
)
self.tf_listener = tf2_ros.TransformListener(
    self.tf_buffer,
    self,
)

script_dir = Path(__file__).resolve().parent

self.coordinate_transformer = RgbdPixelToBase(
    tf_buffer=self.tf_buffer,
    transform_path=str(script_dir / "T_gripper2camera.npy"),
    base_frame="base_link",
    calibration_frame="link_6",
    transform_direction="gripper_to_camera",
    transform_translation_unit="auto",
)


# YOLO 바운딩박스 중심을 사용하는 경우
center_u, center_v = bbox_center_xyxy(
    x_min,
    y_min,
    x_max,
    y_max,
)

# MediaPipe 등에서 중심 픽셀을 이미 계산했다면 그대로 사용
# center_u = detected_center_u
# center_v = detected_center_v

result = self.coordinate_transformer.pixel_to_base(
    u=center_u,
    v=center_v,
    depth_image=self.latest_depth_image,
    depth_encoding=self.latest_depth_encoding,
    camera_info=self.latest_camera_info,
)

if result is None:
    self.get_logger().warning("Depth 또는 TF 문제로 좌표 변환 실패")
else:
    x_mm, y_mm, z_mm = result.base_point_mm

    self.get_logger().info(
        f"object={class_name}, "
        f"base=[{x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}] mm"
    )

    # DB 저장용 문서 예시
    object_document = {
        "class_name": class_name,
        "position": {
            "x_mm": float(x_mm),
            "y_mm": float(y_mm),
            "z_mm": float(z_mm),
        },
        "frame_id": "base_link",
        "depth_m": float(result.depth_m),
    }
"""


if __name__ == "__main__":
    print(__doc__)
    print(INTEGRATION_EXAMPLE)
