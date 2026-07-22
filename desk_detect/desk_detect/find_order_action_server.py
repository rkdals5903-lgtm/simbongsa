"""
FindOrder 액션 서버.
Goal: target(찾을 물체의 YOLO 클래스명) 수신
동작: 찾아질 때까지 최신 프레임에서 반복 추론, 매 시도마다 candidate 좌표를
      TargetPoint[] feedback으로 publish. 찾으면 성공 종료(is_found=True),
      타임아웃되면 실패 종료(is_found=False).

full_scan(전체 테이블 스캔, VoiceKeyword.srv 대응)은 옆 팀원 인터페이스 확정 후
별도로 붙일 예정 -> 지금은 run_inference_on_latest_frame()에 재사용 가능하도록
로직만 공용 메서드로 분리해둠.
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
# from hey_doopal_msg.srv import VoiceKeyword
import message_filters


# 실제 인터페이스 패키지명에 맞게 수정 (스크린샷 기준 hey_doopal_msg)
from hey_doopal_msg.action import FindOrder
from hey_doopal_msg.msg import TargetPoint


class FindOrderActionServer(Node):
    def __init__(
        self,
        model,
        color_topic='/camera/camera/color/image_raw',
        depth_topic='/camera/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/camera/color/camera_info',
        db_api_url='http://localhost:8000/api/objects/',  # Django DB API 엔드포인트로 수정
        find_target_conf=0.5,
        find_target_timeout=5.0,      # 초. 이 시간 안에 못 찾으면 is_found=False
        find_target_interval=0.15,    # 재시도 간격(초)
    ):
        super().__init__('find_order_action_server')
        self.model = model
        self.classNames = model.names
        self.bridge = CvBridge()
        self.db_api_url = db_api_url
        self.find_target_conf = find_target_conf
        self.find_target_timeout = find_target_timeout
        self.find_target_interval = find_target_interval

        #카메라 내부 파라미터 초기값을 None으로 세팅. 아직 camera_info 토픽을 못 받았다.
        self.fx = self.fy = self.cx_intr = self.cy_intr = None

        # camera_info_topic을 구독. 
        # 메시지 오면 self.camera_info_callback 함수가 실행됨. 버퍼 얼마나 쌓을 지 큐 사이즈 = 10
        self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # 최신 프레임 저장용 (action execute_callback과 별도 스레드에서 갱신됨)
        self.latest_color = None
        self.latest_depth = None
        self.frame_lock = threading.Lock()

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.frame_callback)
        # frame_callback 카메라 프레임 올 때마다 (보통 초당 30번) 계속 호출 (구독)

        # execute_callback이 sleep 하며 대기하는 동안 frame_callback도 돌아야 하므로
        # ReentrantCallbackGroup + MultiThreadedExecutor 필수
        action_cb_group = ReentrantCallbackGroup()
        # 액션 서버
        self._action_server = ActionServer(
            self,
            FindOrder,
            'find_order',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            # cancel_callback=self.cancel_callback,
            callback_group=action_cb_group,
        )

        self.get_logger().info('FindOrderActionServer 준비 완료. goal 대기 중...')

    # ---------- 프레임 수신 ----------
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

    # ---------- 좌표 변환 유틸 ----------
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

    # ---------- 공용 추론 메서드 (find_target, full_scan 사용) ----------
    def run_inference_on_latest_frame(self, conf_threshold, target_label=None):
        with self.frame_lock:
            if self.latest_color is None or self.latest_depth is None:
                return []
            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()

        if self.fx is None:
            return []
        # 프레임이 아직 하나도 안 왔으면 빈 리스트 반환

        results = self.model(color_img, verbose=False)
        detections = []

        for r in results:
            # boxes 탐지된 물체들
            for box in r.boxes:
                confidence = math.ceil((box.conf[0] * 100)) / 100
                # 소수점 2자리로 반올림(올림)하고, threshold보다 낮으면 이 박스는 건너뜀 continue
                if confidence < conf_threshold:
                    continue

                cls = int(box.cls[0])
                label = self.classNames.get(cls, f'class_{cls}')
                if target_label and label != target_label:
                    continue
                # 클래스 번호 → 라벨 이름 변환. 
                # target_label이 지정돼 있는데(예: 'airpods') 이 박스의 라벨이 그거랑 다르면 스킵 — "특정 물체만 찾기" 필터링 로직


                # bbox 좌표 꺼내서 depth 읽고, 3D로 변환. 둘 중 하나라도 실패하면(가려짐/노이즈 등) 이 박스는 버림.
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
                # 성공한 것들만 딕셔너리로 모아서 리스트로 반환. 


        return detections

    # ---------- 액션 콜백 ----------
    def goal_callback(self, goal_request):
        self.get_logger().info(f'목표 수신: target="{goal_request.target}"')
        if not goal_request.target:
            self.get_logger().warn('target이 비어있는 goal -> 거절')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # 클라이언트가 직접 취소 요청 할 경우의 처리 TODO
    # def cancel_callback(self, goal_handle):
    #     self.get_logger().info('취소 요청 수신')
    #     return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_label = goal_handle.request.target
        feedback_msg = FindOrder.Feedback()
        result = FindOrder.Result()

        start = time.time()
        attempts = 0
        last_detections = []

        # "한 프레임 실패 = 일시적으로 못 봤을 수도 있다"
        # 여러 프레임에 걸쳐 재시도
        # 사용자가 언제까지 기다려야 하는지 알아야 함, 그리고 진짜로 그 물체가 책상에 없을 수도 있음 TODO 
        # 타임아웃(기본 5초)
        while time.time() - start < self.find_target_timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.is_found = False
                self.get_logger().info(f'"{target_label}" 탐색 취소됨 ({attempts}번 시도)')
                return result

            attempts += 1
            detections = self.run_inference_on_latest_frame(
                self.find_target_conf, target_label=target_label)

            # feedback: 이번 프레임에서 잡힌 후보 좌표들 전송
            points = []
            for d in detections:
                p = TargetPoint()
                p.x, p.y, p.z = d['x'], d['y'], d['z']
                points.append(p)
            feedback_msg.coordinates = points
            goal_handle.publish_feedback(feedback_msg)

            if detections:
                last_detections = detections
                break

            time.sleep(self.find_target_interval)

        found = bool(last_detections)

        if found:
            self.upsert_to_db(last_detections)
            goal_handle.succeed()
            self.get_logger().info(f'"{target_label}" {attempts}번 시도 만에 발견')
        else:
            goal_handle.abort()
            self.get_logger().warn(
                f'"{target_label}" {self.find_target_timeout}초 동안 못 찾음 ({attempts}번 시도)')

        result.is_found = found
        return result

    # ---------- DB upsert (find_target에서 발견 시 위치 갱신용) ----------
    def upsert_to_db(self, detections):
        for obj in detections:
            try:
                check = requests.get(
                    self.db_api_url, params={'class_name': obj['class_name']}, timeout=2.0)
                check.raise_for_status()
                existing = check.json()

                if existing:
                    obj_id = existing[0]['id']  # API 응답 스키마에 맞게 조정
                    res = requests.patch(f"{self.db_api_url}{obj_id}/", json=obj, timeout=2.0)
                else:
                    res = requests.post(self.db_api_url, json=obj, timeout=2.0)

                if res.status_code not in (200, 201):
                    self.get_logger().error(
                        f'DB 저장 실패 ({res.status_code}): {obj["class_name"]}')

            except requests.exceptions.RequestException as e:
                self.get_logger().error(f'DB 요청 실패: {e}')


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
    node = FindOrderActionServer(model)

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