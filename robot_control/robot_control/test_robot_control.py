import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient

from hey_doopal_msg.action import FindTargetOrder
from robot_control.cone_scan import ConeScanner
from rclpy.executors import MultiThreadedExecutor

import DR_init


# =========================
# Robot Configuration
# =========================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 60
ACCELERATION = 60

# =========================
# Scan Configuration
# =========================

SCAN_TILT_ANGLE = 30.0
SCAN_POINT_COUNT = 8

SCAN_VELOCITY = [20, 10]
SCAN_ACCELERATION = [40, 20]

# =========================
# Position Configuration
# =========================

CONFIG = {
    "POS1": [744.33, -127.61, 228.65, 169.90,-142.46, 97.08],
    "POS2": [342.82, 171.74, 395.16, 26.39, -176.26, -1.66],
    "HAND_SCAN": [406.06, -167.10, 555.08, 95.52, -104.05, 24.04],
    "TARGET_SCAN": [684.13, -25.36, 235.69, 169.25, -140.48, 159.67],
}

# 1. DSR 정보 등록
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 2. ROS 초기화
rclpy.init()

# 3. DSR_ROBOT2가 사용할 노드 생성
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movel,
        get_current_posx,
        DR_TOOL,
        DR_BASE,
    )
    from DR_common2 import posx
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()


class TargetScanNode(Node):

    def __init__(self):
        super().__init__("target_scan_node")

        self.target_name = None

        self.find_target_order_client = ActionClient(
            self,
            FindTargetOrder,
            "/find_target_order",
        )

        self.cone_scanner = ConeScanner(
            node=self,
            action_client=self.find_target_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )
        self.scan_thread = None

        self.subscription = self.create_subscription(
            String,
            "/target_command",
            self.command_callback,
            10,
        )

        self.get_logger().info("target_scan_node 시작")
    def run_cone_scan(
        self,
        center_pose,
        target_name,
    ):
        coordinate = self.cone_scanner.scan(
            center_pose,
            target_name,
        )

        if coordinate is None:
            self.get_logger().warning(
                f"{target_name} 좌표 탐색 실패"
            )
            return

        self.get_logger().info(
            f"{target_name} 최종 좌표: {coordinate}"
        )

    def command_callback(self, msg):
        target_name = msg.data.strip()

        if not target_name:
            self.get_logger().warning(
                "빈 메시지를 받았습니다."
            )
            return

        if (
            self.scan_thread is not None
            and self.scan_thread.is_alive()
        ):
            self.get_logger().warning(
                "이미 스캔이 진행 중입니다."
            )
            return

        self.target_name = target_name

        if target_name == "hand":
            center_pose = CONFIG["HAND_SCAN"]
        else:
            center_pose = CONFIG["HAND_SCAN"]

        self.get_logger().info(
            f"스캔 명령 수신: {target_name}"
        )

        # 구독 콜백은 바로 끝내고,
        # 실제 스캔은 별도 스레드에서 실행한다.
        self.scan_thread = threading.Thread(
            target=self.run_cone_scan,
            args=(center_pose, target_name),
            daemon=True,
        )
        self.scan_thread.start()


    # def command_callback(self, msg):
    #     message = msg.data.strip()

    #     if not message:
    #         self.get_logger().warn("빈 메시지를 받았습니다.")
    #         return

    #     parts = message.split(maxsplit=1)

    #     # 첫 번째 값은 타겟 이름
    #     self.target_name = parts[0]
    #     self.goal_name = parts[1] if len(parts) > 1 else None

    #     # 타겟 이름만 받은 경우
    #     if len(parts) == 1:
    #         self.get_logger().info(f"타겟 이름만 수신: {self.target_name}")
    #         position_key = CONFIG["HAND_SCAN"]
    #         target_name = "hand"
    #         goal_coordinate = self.cone_scanner.scan(position_key, target_name)

    #         # target_coordinate = self.get_coordinate(self.target_name)
    #         if not target_coordinate:
    #             position_key = CONFIG["TARGET_SCAN"]
    #             target_coordinate = self.cone_scanner.scan(position_key, self.target_name)

    #         # self.pick_and_place_target(target_coordinate, goal_coordinate)
            
    #     # 타겟 이름과 추가 값이 있는 경우
    #     else:
    #         # goal_coordinate = self.get_coordinate(self.goal_name)
    #         # target_coordinate = self.get_coordinate(self.target_name)
    #         if not target_coordinate:
    #             position_key = CONFIG["TARGET_SCAN"]
    #             target_coordinate = self.cone_scanner.scan(position_key, self.target_name)


    #         # self.pick_and_place_target(target_coordinate, goal_coordinate)

    #         self.get_logger().info(f"타겟 이름 수신: {self.target_name}")
    #         self.get_logger().info(f"추가 값 수신: {parts[1]}")


    # def cone_scan(self, center_pose):
    #     scan_pose_list = []

    #     self.get_logger().info("원뿔 스캔 시작")

    #     center_pos = posx(*center_pose)

    #     for index in range(SCAN_POINT_COUNT):
    #         theta = (
    #             2.0
    #             * math.pi
    #             * index
    #             / SCAN_POINT_COUNT
    #         )

    #         tilt_rx = SCAN_TILT_ANGLE * math.cos(theta)
    #         tilt_ry = SCAN_TILT_ANGLE * math.sin(theta)

    #         rotation_delta = [
    #             0.0,
    #             0.0,
    #             0.0,
    #             tilt_rx,
    #             tilt_ry,
    #             0.0,
    #         ]

    #         # center_pose와 rotation_delta를 원소별로 더한다.
    #         scan_pose_values = [
    #             center_value + delta_value
    #             for center_value, delta_value
    #             in zip(center_pose, rotation_delta)
    #         ]

    #         # 더한 좌표를 두산 작업좌표 posx로 변환한다.
    #         scan_pose = posx(*scan_pose_values)

    #         scan_pose_list.append(scan_pose)

    #         self.get_logger().info(
    #             f"scan_pose[{index}]: {scan_pose}"
    #         )

    #     # 계산한 자세로 하나씩 이동
    #     for index, scan_pose in enumerate(scan_pose_list):
    #         movel(
    #             scan_pose,
    #             vel=SCAN_VELOCITY,
    #             acc=SCAN_ACCELERATION,
    #             ref=DR_BASE,
    #         )

    #         self.get_logger().info(
    #             f"scan_pose[{index}] 이동 완료"
    #         )

    #     # 원래 중심 자세로 복귀
    #     movel(
    #         center_pos,
    #         vel=SCAN_VELOCITY,
    #         acc=SCAN_ACCELERATION,
    #         ref=DR_BASE,
    #     )

    #     self.get_logger().info("원뿔 스캔 완료")

    def move_to_position(self, position_key):
        if position_key not in CONFIG:
            self.get_logger().warn(f"등록되지 않은 위치 키입니다: {position_key}")
            return

        # 키를 입력해서 실제 좌표 값을 가져온다.
        target_position = CONFIG[position_key]

        self.get_logger().info(f"위치 키: {position_key}")
        self.get_logger().info(f"이동 좌표: {target_position}")

        movel(target_position, vel=VELOCITY, acc=ACCELERATION,)

        self.get_logger().info(f"{position_key} 이동 완료")

def main(args=None):
    node = TargetScanNode()

    # 중요:
    # rclpy.spin(node)의 기본 global executor를 사용하지 않는다.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info("노드를 종료합니다.")

    finally:
        executor.shutdown()

        node.destroy_node()
        dsr_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()