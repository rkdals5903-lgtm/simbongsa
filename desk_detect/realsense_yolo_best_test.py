#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RealSense ROS2 RGB 영상에서 사용자 학습 YOLO best.pt 테스트

기본 설정
---------
모델:
    /home/rokey/mediapipe_test/best.pt

RGB 토픽:
    /camera/camera/color/image_raw

기능
----
- RealSense RGB 영상을 ROS2로 구독
- Ultralytics YOLO best.pt로 실시간 객체 탐지
- Bounding box, class, confidence 표시
- FPS 및 탐지 개수 표시
- S 키: 원본/탐지 결과 이미지 저장
- Q 또는 ESC: 종료
- 클래스별 confidence threshold 오버라이드 지원 (PER_CLASS_CONF_THRESHOLD)  # ★ 신규 기능

실행 예시
---------
python3 realsense_yolo_best_test.py \
  --ros-args \
  -p model_path:=/home/rokey/cobot_ws/src/simbongsa/desk_detect/my_best_roboflow.pt \
  -p confidence:=0.5
"""

import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise ImportError(
        "ultralytics가 설치되어 있지 않습니다.\n"
        "설치 명령어: pip install ultralytics"
    ) from exc


# ============================================================
# ★ 신규 추가: 클래스별 confidence threshold 오버라이드
#   지정 안 한 클래스는 --ros-args -p confidence:=... 값(기본 0.5) 그대로 사용.
#   예: airpods만 오탐이 많아서 더 엄격하게 걸러내고 싶은 경우
# ============================================================
PER_CLASS_CONF_THRESHOLD = {
    'airpods': 0.9,
}


class RealSenseYoloTester(Node):
    def __init__(self) -> None:
        super().__init__("realsense_yolo_best_tester")

        # ROS2 파라미터
        self.declare_parameter(
            "model_path",
            "/home/rokey/cobot_ws/src/simbongsa/desk_detect/my_best_roboflow.pt",
        )
        self.declare_parameter(
            "color_topic",
            "/camera/camera/color/image_raw",
        )
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_det", 100)
        self.declare_parameter("mirror_view", False)
        self.declare_parameter(
            "save_dir",
            "/home/rokey/mediapipe_test/yolo_test_results",
        )

        self.model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()
        self.color_topic = str(
            self.get_parameter("color_topic").value
        )
        self.confidence = float(
            self.get_parameter("confidence").value
        )
        self.iou = float(
            self.get_parameter("iou").value
        )
        self.imgsz = int(
            self.get_parameter("imgsz").value
        )
        self.device = str(
            self.get_parameter("device").value
        )
        self.max_det = int(
            self.get_parameter("max_det").value
        )
        self.mirror_view = bool(
            self.get_parameter("mirror_view").value
        )
        self.save_dir = Path(
            str(self.get_parameter("save_dir").value)
        ).expanduser()

        # ★ 신규 추가: 클래스별 threshold 오버라이드 (전역 상수를 그대로 인스턴스에 보관)
        self.per_class_conf_threshold = PER_CLASS_CONF_THRESHOLD

        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"YOLO 모델 파일을 찾을 수 없습니다: {self.model_path}"
            )

        self.save_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.get_logger().info(
            f"YOLO 모델 로딩 중: {self.model_path}"
        )
        self.model = YOLO(str(self.model_path))

        self.class_names = self.model.names
        self.get_logger().info(
            f"학습 클래스: {self.class_names}"
        )
        # ★ 신규 추가: 클래스별 threshold 오버라이드 로그
        if self.per_class_conf_threshold:
            self.get_logger().info(
                f"클래스별 threshold 오버라이드: {self.per_class_conf_threshold}"
            )

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()

        self.latest_frame: Optional[np.ndarray] = None
        self.latest_frame_seq = 0
        self.latest_receive_time = 0.0

        self.latest_original: Optional[np.ndarray] = None
        self.latest_annotated: Optional[np.ndarray] = None

        self.subscription = self.create_subscription(
            Image,
            self.color_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"RealSense RGB 구독: {self.color_topic}"
        )
        self.get_logger().info(
            (
                f"confidence={self.confidence:.2f}, "
                f"iou={self.iou:.2f}, "
                f"imgsz={self.imgsz}, "
                f"device={self.device}"
            )
        )
        self.get_logger().info(
            "Q/ESC: 종료, S: 결과 이미지 저장"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as exc:
            self.get_logger().error(
                f"RGB 영상 변환 실패: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        with self.frame_lock:
            self.latest_frame = frame.copy()
            self.latest_frame_seq += 1
            self.latest_receive_time = time.monotonic()

    def get_latest_frame(
        self,
        last_processed_seq: int,
    ):
        with self.frame_lock:
            if (
                self.latest_frame is None
                or self.latest_frame_seq == last_processed_seq
            ):
                return None, last_processed_seq

            return (
                self.latest_frame.copy(),
                self.latest_frame_seq,
            )

    # ============================================================
    # ★ 신규 추가 메서드 (기존 파일에는 없었음)
    # ============================================================
    def filter_boxes_by_class_threshold(self, result):
        """
        model.predict(conf=...)는 클래스 구분 없이 단일 threshold만 적용하므로,
        여기서 클래스별 threshold를 한 번 더 적용해서 통과하는 박스 인덱스만 골라냄.
        per_class_conf_threshold에 없는 클래스는 이미 predict 단계에서
        self.confidence로 걸러진 상태 그대로 통과.
        """
        if not self.per_class_conf_threshold or result.boxes is None:
            return result

        keep_indices = []
        for i, box in enumerate(result.boxes):
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            class_name = self.class_names.get(class_id, str(class_id))

            threshold = self.per_class_conf_threshold.get(class_name, self.confidence)
            if confidence >= threshold:
                keep_indices.append(i)

        result.boxes = result.boxes[keep_indices]
        return result

    def run_inference(
        self,
        frame: np.ndarray,
    ) -> np.ndarray:
        inference_frame = (
            cv2.flip(frame, 1)
            if self.mirror_view
            else frame
        )

        started_at = time.perf_counter()

        results = self.model.predict(
            source=inference_frame,
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.imgsz,
            device=self.device,
            max_det=self.max_det,
            verbose=False,
        )

        elapsed = time.perf_counter() - started_at
        fps = 1.0 / elapsed if elapsed > 0.0 else 0.0

        result = results[0]
        result = self.filter_boxes_by_class_threshold(result)  # ★ 신규 추가: 클래스별 threshold 추가 필터링
        annotated = result.plot()

        detection_count = (
            0
            if result.boxes is None
            else len(result.boxes)
        )

        cv2.putText(
            annotated,
            f"Detections: {detection_count}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"Inference FPS: {fps:.1f}",
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"Confidence: {self.confidence:.2f}",
            (20, 101),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                class_name = self.class_names.get(
                    class_id,
                    str(class_id),
                )

                xyxy = box.xyxy[0].tolist()
                x1, y1, x2, y2 = [
                    int(value)
                    for value in xyxy
                ]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                cv2.circle(
                    annotated,
                    (center_x, center_y),
                    5,
                    (0, 0, 255),
                    -1,
                )
                cv2.putText(
                    annotated,
                    f"center=({center_x},{center_y})",
                    (
                        max(0, x1),
                        min(
                            annotated.shape[0] - 10,
                            y2 + 22,
                        ),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

                self.get_logger().info(
                    (
                        f"검출: {class_name}, "
                        f"conf={confidence:.3f}, "
                        f"center=({center_x}, {center_y})"
                    ),
                    throttle_duration_sec=0.5,
                )

        self.latest_original = inference_frame.copy()
        self.latest_annotated = annotated.copy()

        return annotated

    def make_waiting_screen(
        self,
    ) -> np.ndarray:
        image = np.zeros(
            (480, 848, 3),
            dtype=np.uint8,
        )

        cv2.putText(
            image,
            "WAITING FOR REALSENSE RGB...",
            (95, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            self.color_topic,
            (90, 265),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        return image

    def save_current_images(self) -> None:
        if (
            self.latest_original is None
            or self.latest_annotated is None
        ):
            self.get_logger().warning(
                "아직 저장할 탐지 결과가 없습니다."
            )
            return

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        original_path = (
            self.save_dir
            / f"{timestamp}_original.jpg"
        )
        annotated_path = (
            self.save_dir
            / f"{timestamp}_detected.jpg"
        )

        original_ok = cv2.imwrite(
            str(original_path),
            self.latest_original,
        )
        annotated_ok = cv2.imwrite(
            str(annotated_path),
            self.latest_annotated,
        )

        if original_ok and annotated_ok:
            self.get_logger().info(
                (
                    "이미지 저장 완료:\n"
                    f"  원본: {original_path}\n"
                    f"  결과: {annotated_path}"
                )
            )
        else:
            self.get_logger().error(
                "이미지 저장에 실패했습니다."
            )

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[RealSenseYoloTester] = None
    executor: Optional[MultiThreadedExecutor] = None
    executor_thread: Optional[threading.Thread] = None

    try:
        node = RealSenseYoloTester()

        executor = MultiThreadedExecutor(
            num_threads=2
        )
        executor.add_node(node)

        executor_thread = threading.Thread(
            target=executor.spin,
            daemon=True,
            name="ros2_executor",
        )
        executor_thread.start()

        # OpenCV GUI와 YOLO 추론은 메인 스레드에서 수행한다.
        last_processed_seq = -1
        display_image = node.make_waiting_screen()

        while rclpy.ok():
            frame, frame_seq = node.get_latest_frame(
                last_processed_seq
            )

            if frame is not None:
                last_processed_seq = frame_seq
                display_image = node.run_inference(
                    frame
                )
            elif (
                time.monotonic()
                - node.latest_receive_time
                > 1.0
            ):
                display_image = (
                    node.make_waiting_screen()
                )

            cv2.imshow(
                "RealSense YOLO best.pt Test",
                display_image,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                break

            if key in (ord("s"), ord("S")):
                node.save_current_images()

            time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(
            f"실행 오류: {exc}",
            file=sys.stderr,
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()

        if executor is not None:
            executor.shutdown()

        if (
            executor_thread is not None
            and executor_thread.is_alive()
        ):
            executor_thread.join(timeout=1.0)

        if node is not None:
            node.destroy_node()


if __name__ == "__main__":
    main()