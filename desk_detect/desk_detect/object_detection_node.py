"""
비전 노드 (책상 전체 스캔 + 특정 물체 찾기) 통합
카메라 구독/좌표변환/DB 로직은 공용으로 한 번만 두고,
그 위에 두 가지 입구(서비스+액션)를 얹음:

  - ScanRequest 서비스 ('yolo_scan_request')
      -> 책상 전체 스캔. 로봇제어가 웨이포인트마다 도착할 때 1번씩 호출.
        1프레임 인식 + DB upsert 후 바로 응답.

  - FindOrder 액션 ('find_order')
      -> 에어팟 등 특정 물체 찾기. goal(target_name) 받으면 계속 최신 프레임 확인,
        state로 진행상황 feedback, 찾으면 result(found, coordinate, message) 반환.
"""

import math
import sys
import time
import threading
import requests
import numpy as np
from pathlib import Path
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import message_filters

#  TODO 실제 인터페이스 패키지명에 맞게 수정
from hey_doopal_msg.srv import ScanRequest   # ← 서비스 타입 (신규)
from hey_doopal_msg.action import FindOrder  # ← 액션 타입 (기존)


class ObjectDetectionNode(Node):
    def __init__(
        self,
        model,
        color_topic='/camera/camera/color/image_raw',
        depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/camera/color/camera_info',
        db_api_url='http://localhost:8000/api/objects/',  # Django DB API 엔드포인트로 수정
        full_scan_conf=0.6,
        find_target_conf=0.5,
        find_target_timeout=10.0,   # 초. 이 시간 안에 못 찾으면 is_found=False
        find_target_interval=0.15,  # 재시도 간격(초)
    ):
        super().__init__('object_detection_node')
        self.model = model
        self.classNames = model.names
        self.bridge = CvBridge()
        self.db_api_url = db_api_url
        self.full_scan_conf = full_scan_conf            # 전체 스캔용 
        self.find_target_conf = find_target_conf        # 타겟 스캔용 
        self.find_target_timeout = find_target_timeout
        self.find_target_interval = find_target_interval

        # ---------- 카메라 관련 (공용) ----------
        self.fx = self.fy = self.cx_intr = self.cy_intr = None
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.latest_color = None
        self.latest_depth = None
        self.frame_lock = threading.Lock()

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.frame_callback)

        # ---------- 서비스 서버: 전체 스캔 ----------
        service_cb_group = ReentrantCallbackGroup()
        self.scan_srv = self.create_service(
            ScanRequest, 'yolo_scan_request',
            self.handle_scan_request, callback_group=service_cb_group)
        
        # 스캔 요청 오면 실행 할 함수 handle_scan_request

        # ---------- 액션 서버: 타겟 찾기 ----------
        action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FindOrder,
            'find_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_cb_group,
        )

        self.get_logger().info(
            'ObjectDetectionNode 준비 완료. yolo_scan_request(서비스), find_order(액션) 대기 중...')

    # ================= 카메라 수신 (공용) =================
    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx_intr = msg.k[2]
        self.cy_intr = msg.k[5]

    def frame_callback(self, color_msg: Image, depth_msg: Image):
        color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        with self.frame_lock:
            self.latest_color = color_img
            self.latest_depth = depth_img

    # ================= 좌표 변환 유틸 (공용) =================
    def pixel_to_3d(self, u, v, depth_m):
        if depth_m <= 0.0 or self.fx is None:
            return None
        x = (u - self.cx_intr) * depth_m / self.fx
        y = (v - self.cy_intr) * depth_m / self.fy
        return float(x), float(y), float(depth_m)

    def get_depth_at_bbox(self, depth_img, x1, y1, x2, y2):
        h, w = depth_img.shape[:2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        half = max(2, (x2 - x1) // 8)
        xs = slice(max(0, cx - half), min(w, cx + half))
        ys = slice(max(0, cy - half), min(h, cy + half))
        patch = depth_img[ys, xs].astype(np.float32)
        valid = patch[patch > 0]
        if valid.size == 0:
            return None, cx, cy
        return float(np.median(valid)) / 1000.0, cx, cy

    def run_inference_on_latest_frame(self, conf_threshold, target_label=None):
        """target_label=None -> 전체 스캔, 지정하면 그 라벨만 필터링 (타겟 찾기)"""
        with self.frame_lock:
            if self.latest_color is None or self.latest_depth is None:
                return []
            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()

        if self.fx is None:
            return []

        results = self.model(color_img, verbose=False)
        detections = []

        for r in results:
            for box in r.boxes:
                confidence = math.ceil((box.conf[0] * 100)) / 100
                if confidence < conf_threshold:
                    continue

                cls = int(box.cls[0])
                label = self.classNames.get(cls, f'class_{cls}')
                # 내가 찾는 라벨이 아니면 continue 에서 거름 (라벨이 다르면 버린다.)
                if target_label and label != target_label:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                depth_m, cx, cy = self.get_depth_at_bbox(depth_img, x1, y1, x2, y2)
                if depth_m is None:
                    continue

                point_3d = self.pixel_to_3d(cx, cy, depth_m)
                if point_3d is None:
                    continue

                x, y, z = point_3d
                detections.append({
                    'class_name': label,
                    'confidence': confidence,
                    'x': round(x, 4),
                    'y': round(y, 4),
                    'z': round(z, 4),
                })

        return detections

    # ================= DB upsert (공용) =================
    def upsert_to_db(self, detections):
        for obj in detections:
            try:
                check = requests.get(
                    self.db_api_url, params={'class_name': obj['class_name']}, timeout=2.0)
                check.raise_for_status()
                existing = check.json()

                if existing:
                    obj_id = existing[0]['id']
                    res = requests.patch(f"{self.db_api_url}{obj_id}/", json=obj, timeout=2.0)
                else:
                    res = requests.post(self.db_api_url, json=obj, timeout=2.0)

                if res.status_code not in (200, 201):
                    self.get_logger().error(
                        f'DB 저장 실패 ({res.status_code}): {obj["class_name"]}')

            except requests.exceptions.RequestException as e:
                self.get_logger().error(f'DB 요청 실패: {e}')

    # ================= 서비스: 전체 스캔 =================
    def handle_scan_request(self, request, response):
        self.get_logger().info(f'[전체 스캔] 요청 수신 (waypoint_id="{request.waypoint_id}")')

        detections = self.run_inference_on_latest_frame(self.full_scan_conf)
        # target_label 안넘김 — 즉 None(기본값)이라 전체 클래스 다 탐지하는 모드로 호출.

        if detections:
            # detections 리스트에 뭐라도 들어있으면 (물체를 1개 이상 찾음)
            self.upsert_to_db(detections)
            response.success = True
            response.message = f'{len(detections)}개 객체 스캔 및 DB 저장 완료'
            response.detected_count = len(detections)
        else:
            # detections가 빈 리스트([])면 (이번 프레임엔 아무것도 안 보임)
            response.success = True
            response.message = '탐지된 객체 없음'
            response.detected_count = 0

        self.get_logger().info(f'[전체 스캔] 응답: {response.message}')
        return response

    # ================= 액션: 타겟 찾기 =================
    def goal_callback(self, goal_request):
        self.get_logger().info(f'[타겟 찾기] 목표 수신: target_name="{goal_request.target_name}"')
        if not goal_request.target_name:
            self.get_logger().warn('[타겟 찾기] target_name이 비어있는 goal -> 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('[타겟 찾기] 취소 요청 수신')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_label = goal_handle.request.target_name
        feedback_msg = FindOrder.Feedback()
        result = FindOrder.Result()

        start = time.time()
        attempts = 0
        found_detection = None

        while time.time() - start < self.find_target_timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.found = False
                result.coordinate = [0.0] * 6
                result.message = ''
                self.get_logger().info(f'[타겟 찾기] "{target_label}" 취소됨 ({attempts}번 시도)')
                return result

            attempts += 1
            detections = self.run_inference_on_latest_frame(
                self.find_target_conf, target_label=target_label)

            feedback_msg.state = (
                f'"{target_label}" 발견됨 (시도 {attempts}회)' if detections
                else f'탐색 중... (시도 {attempts}회)')
            goal_handle.publish_feedback(feedback_msg)

            if detections:
                found_detection = max(detections, key=lambda d: d['confidence'])
                break

            time.sleep(self.find_target_interval)

        if found_detection:
            self.upsert_to_db([found_detection])
            goal_handle.succeed()

            x, y, z = found_detection['x'], found_detection['y'], found_detection['z']
            result.found = True
            # coordinate는 배열 길이 6 고정(인터페이스 스펙)이지만, 지금은 x,y,z 3개만 실제 값
            result.coordinate = [x, y, z, 0.0, 0.0, 0.0]
            result.message = found_detection['class_name']
            self.get_logger().info(f'[타겟 찾기] "{target_label}" {attempts}번 시도 만에 발견')
        else:
            goal_handle.abort()
            result.found = False
            result.coordinate = [0.0] * 6
            result.message = ''
            self.get_logger().warn(
                f'[타겟 찾기] "{target_label}" {self.find_target_timeout}초 동안 못 찾음 '
                f'({attempts}번 시도)')

        return result


def main():
    model_path = '/home/rokey/ros2_ws/src/simbongsa/desk_detect/my_best_roboflow.pt'  # 학습된 모델 경로로 수정

    if not Path(model_path).exists():
        print(f'File not found: {model_path}')
        sys.exit(1)

    suffix = Path(model_path).suffix.lower()
    if suffix == '.pt':
        model = YOLO(model_path)
    elif suffix in ['.onnx', '.engine']:
        model = YOLO(model_path, task='detect')
    else:
        print(f'Unsupported model format: {suffix}')
        sys.exit(1)

    rclpy.init()
    node = ObjectDetectionNode(model)

    # 서비스/액션 콜백이 서로, 그리고 frame_callback과 동시에 돌아야 하므로 MultiThreadedExecutor 필수
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('Ctrl+C received. Exiting...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()