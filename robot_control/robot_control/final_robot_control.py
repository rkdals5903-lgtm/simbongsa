import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy # [ADD] QoS 임포트

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from std_msgs.msg import String
from hey_doopal_msg.action import FindOrder
from hey_doopal_msg.srv import ScanRequest
from hey_doopal_msg.srv import VoiceKeyword
from hey_doopal_msg.srv import GripBoundingBox
from hey_doopal_msg.srv import GetFixedPose
from hey_doopal_msg.srv import GetScanCase

from robot_control.cone_scan import ConeScanner
from rclpy.executors import MultiThreadedExecutor

import DR_init

# =========================
# Gripper Configuration
# =========================

GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502

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

# YOLO 서비스가 나타날 때까지 기다리는 시간
YOLO_SERVICE_WAIT_TIMEOUT = 5.0

# YOLO 한 번의 스캔 응답을 기다리는 시간
YOLO_SCAN_RESPONSE_TIMEOUT = 30.0

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
        movej,
        get_current_posx,
        task_compliance_ctrl,
        set_desired_force,
        get_tool_force,
        release_force,
        release_compliance_ctrl,
        set_ref_coord,
        DR_FC_MOD_REL,
        DR_BASE,
        )
    
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

# ##############################################################################
# [ADD] GRIPPER CONTROL CLASS START
# ##############################################################################
class AdaptiveGripper:
    def __init__(self, ip, port=502):
        self.client = ModbusClient(ip, port=port, stopbits=1, bytesize=8, parity="E", baudrate=115200, timeout=1)
        self.client.connect()
        self.max_width = 1100

    def move_gripper(self, width_val, force_val):
        params = [force_val, width_val, 16] 
        self.client.write_registers(address=0, values=params, unit=65)

    def get_status(self):
        result = self.client.read_holding_registers(address=268, count=1, unit=65)
        status = format(result.registers[0], "016b")
        return [int(status[-2])] # 1이면 grip detected
    def close_connection(self):
        self.client.close()
# ##############################################################################
# [ADD] GRIPPER CONTROL CLASS END
# ##############################################################################

class TargetScanNode(Node):

    def __init__(self):
        super().__init__("robot_control_node")

        self.gripper = AdaptiveGripper(GRIPPER_IP,GRIPPER_PORT) 

        self.scan_thread = None
        self.detected_coordinate = None

        #topic

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # publisher 설정
        self.pub_table_scan = self.create_publisher(Bool, '/table_scan_finished', qos_profile)
        self.pub_hand_start = self.create_publisher(Bool, '/hand_scan_start', qos_profile)
        self.pub_hand_finish = self.create_publisher(Bool, '/hand_scan_finished', qos_profile)
        self.pub_task_completed = self.create_publisher(Bool, '/task_completed', qos_profile)
        self.pub_table_rescan_start = self.create_publisher(Bool, '/table_rescan_started', qos_profile)
        self.pub_table_rescan_finish = self.create_publisher(Bool, '/table_rescan_finished', qos_profile)
        self.pub_say = self.create_publisher(String, '/say', qos_profile)
        self.pub_error_status = self.create_publisher(String, '/robot_error_status', qos_profile)
        
        # service_client
        self.scan_table_client= self.create_client(ScanRequest, "/yolo_scan_request")
        self.grip_bbox_client = self.create_client(GripBoundingBox, "/grip_bounding_box")
        self.approach_client = self.create_client(Trigger, "/arrived_goal")

        # service_client(DB)
        self.db_fixed_pose_client = self.create_client(GetFixedPose, "/get_fixed_pose")
        self.db_scan_case_client = self.create_client(GetScanCase, "/get_scan_case")

        # service_server
        self.get_keyword_service = self.create_service(VoiceKeyword, "/get_keyword", self.command_callback)
        self.ungrip_service = self.create_service(Trigger, "/ungrip", self.ungrip_callback)

        # action
        self.find_target_order_client = ActionClient(self, FindOrder, "/find_target_order")
        self.find_hand_order_client = ActionClient(self, FindOrder, "/find_hand_order")        
        
        self.target_scanner = ConeScanner(
            node=self,
            action_client=self.find_target_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )
        self.hand_scanner = ConeScanner(
            node=self,
            action_client=self.find_hand_order_client,
            scan_tilt_angle=SCAN_TILT_ANGLE,
            scan_point_count=SCAN_POINT_COUNT,
            scan_velocity=SCAN_VELOCITY,
            scan_acceleration=SCAN_ACCELERATION,
        )

        self.get_logger().info("target_scan_node 시작")


    # 명령 수신 시 실행되는 콜백 함수
    def command_callback(self, request, response):
        target_names = request.target.strip().split()
        goal_name = request.goal.strip()

        self.get_logger().info(f"명령 수신: target={target_names}, goal={goal_name}")

        if goal_name == "table_scan":
            threading.Thread(
                target=self.run_table_scan,
                args=(goal_name,),
                daemon=True,
            ).start()
            response.accepted = True
            return response

        threading.Thread(
            target=self.execute_robot_task,
            args=(target_names, goal_name),
            daemon=True,
        ).start()
        response.accepted = True
        return response

    # 메인 로직: target 좌표를 DB에서 가져와서 이동 후, goal 좌표로 이동
    def execute_robot_task(self, target_names, goal_name):

        # goal 좌표 설정
        if not goal_name :
            goal_name = "hand"

        response = self.get_object_from_db(goal_name)
        if response is None:
            self.get_logger().error(f"{goal_name} DB 좌표 수신 실패")
            error = String()
            error.data = "DB 좌표 수신 실패"
            self.pub_error_status.publish(error) 
            return
        goal_coordinate = list(response.pose)
                
        if goal_name == "hand":
            hand_scan = Bool()
            hand_scan.data = True
            self.pub_hand_start.publish(hand_scan) 
            center_pose = list(goal_coordinate)
            scan_success = self.run_cone_scan(center_pose, goal_name)
            if not scan_success:
                self.get_logger().error(f"{goal_name} 좌표 탐색 실패")
                error = String()
                error.data = "좌표 탐색 실패"
                self.pub_error_status.publish(error)
                return
            goal_coordinate[:3]= self.detected_coordinate[:3]
            self.pub_hand_finish.publish(hand_scan)
        
        self.get_logger().info(f"목표좌표 설정 완료:{goal_coordinate}")
        
        # target 좌표를 DB에서 가져와서 이동
        for target_name in target_names:

            response = self.get_object_from_db(target_name)
            if response is None:
                self.get_logger().error(f"{target_name} DB 좌표 수신 실패")
                error = String()
                error.data = "DB 좌표 수신 실패"
                self.pub_error_status.publish(error)
                continue
            target_coordinate = list(response.pose)

            self.get_logger().info(f"DB 타깃좌표 설정 완료:{target_coordinate}")

            move_name = f"{target_name}_above"
            target_above = list(target_coordinate)
            target_above[2] += 100.0
            
            move_success = self.move_to_position(move_name, target_above)

            if not move_success:
                self.get_logger().error(f"{move_name} 이동 실패")
                error = String()
                error.data = "이동 실패"
                self.pub_error_status.publish(error)
                continue

            self.gripper.move_gripper(width_val=1000, force_val=200)

            grip_request = GripBoundingBox.Request()
            grip_request.target = target_name

            grip_response = self.call_service(self.grip_bbox_client, grip_request, timeout=30.0)

            if grip_response is None:
                self.get_logger().error("YOLO 파지 정보 수신 실패")
                continue

            self.get_logger().info(
                f"YOLO 파지 정보 수신: "
                f"coordinate={list(grip_response.coordinate)}, "
                f"bbox_width={grip_response.bbox_width}, "
                f"bbox_height={grip_response.bbox_height}, "
                f"depth={grip_response.camera_depth_z}"
                f"is_find={grip_response.is_find}"
            )
            # =========================
            # YOLO 좌표로 이동
            # =========================
            if not grip_response.is_find:
                center_pose = target_above
                target = target_name

                target_scan = Bool()
                target_scan.data = True
                self.pub_table_rescan_start.publish(target_scan) 
                scan_success = self.run_cone_scan(center_pose, target)

                if not scan_success:
                    self.get_logger().error(f"{target_name} 좌표 탐색 실패")
                    target_scan.data = False
                    self.pub_table_rescan_finish.publish(target_scan) 
                    continue
                
                self.pub_table_rescan_finish.publish(target_scan) 
                
                target_coordinate = list(center_pose)
                target_coordinate[:3] = self.detected_coordinate[:3]

            grip_coordinate = list(target_coordinate)
            grip_coordinate[:3] = grip_response.coordinate[:3]
            self.get_logger().info(f"타깃좌표 설정 완료:{grip_coordinate}")

            move_name = f"{target_name}_grip"
            move_success = self.move_to_position(move_name, grip_coordinate)

            if not move_success:
                self.get_logger().error(f"{move_name} 이동 실패")
                continue

            # =========================
            # 물체 파지
            # =========================
            grip_success = self.run_adaptive_grip(
                bbox_w=grip_response.bbox_width,
                bbox_h=grip_response.bbox_height,
                dist=grip_response.camera_depth_z,
            )
            if not grip_success:
                self.get_logger().error("Adaptive Grip 실패")
                error = String()
                error.data = "Adaptive Grip 실패"
                self.pub_error_status.publish(error)
                continue
            # 물체를 잡은 후 위로 이동
            grip_above = list(grip_coordinate)
            grip_above[2] += 100.0

            move_name = f"{target_name}_above_after_grip"
            move_success = self.move_to_position(move_name, grip_above)

            if not move_success:
                self.get_logger().error(f"{move_name} 이동 실패")
                error = String()
                error.data = f"{move_name} 이동 실패"
                self.pub_error_status.publish(error)
                return

            # =========================
            # goal로 이동
            # =========================
            # 손에 전달할 때
            if goal_name == "hand":
                move_name = f"{goal_name}"
                move_success = self.move_to_position(move_name, goal_coordinate)
        
                if not move_success:
                    self.get_logger().error(f"{goal_name} 이동 실패")
                    error = String()
                    error.data = f"{goal_name} 이동 실패"
                    self.pub_error_status.publish(error)
                    return
                else:
                    self.get_logger().info(f"{goal_name} 이동 완료")
                    request = Trigger.Request()
                    approach_response = self.call_service(self.approach_client, request, timeout=5.0)

                    if approach_response is None:
                        self.get_logger().error("arrived_goal 서비스 호출 실패")
                        error = String()
                        error.data = "arrived_goal 서비스 호출 실패"
                        self.pub_error_status.publish(error)
                        return

            # 손이 아닌 경우 "pos1, pos2, pos3"
            else:
                move_name = f"{goal_name}_approach"
                goal_above = list(goal_coordinate)
                goal_above[2] += 10.0

                move_success = self.move_to_position(move_name, goal_above)
                if not move_success:
                    self.get_logger().error(f"{goal_name} 이동 실패")
                    error = String()
                    error.data = f"{goal_name} 이동 실패"
                    self.pub_error_status.publish(error)
                    return
                place_success = self.place_with_compliance()

                if not place_success:
                    self.get_logger().error("내려놓기 실패")
                    error = String()
                    error.data = "내려놓기 실패"
                    self.pub_error_status.publish(error)
                    return

            self.get_logger().info("로봇 작업 완료")

    # 내려놓기 위한 순응 제어
    def place_with_compliance(self, press_force=20.0, force_threshold=8.0, timeout=5.0):
        self.get_logger().info("컴플라이언스 하강 시작")

        contacted = False

        try:
            # ================================
            # 1. Base 기준
            # ================================
            set_ref_coord(DR_BASE)

            force = get_tool_force(DR_BASE)

            self.get_logger().info(f"Force 시작 전: Fx={force[0]:.2f}, Fy={force[1]:.2f}, Fz={force[2]:.2f}")

            # ================================
            # 2. Compliance ON
            # ================================
            ret_compliance = task_compliance_ctrl(stx=[3000, 3000, 500, 200, 200, 200])

            self.get_logger().info(f"Compliance ON, ret={ret_compliance}")
            # 트러블 슈팅 컴플라이언스 이후 0.5초 후 Force 켜기 두산패키지 이슈에 써있었음
            time.sleep(0.5)

            # ================================
            # 3. Force ON
            # ================================
            ret_force = set_desired_force(
                fd=[0, 0, -press_force, 0, 0, 0],
                dir=[0, 0, 1, 0, 0, 0],
                time=0.5,
                mod=DR_FC_MOD_REL,
            )

            self.get_logger().info(f"Force ON: -Z {press_force}N, ret={ret_force}")

            start_time = time.time()

            # ================================
            # 4. 실제 Fz를 직접 확인
            # ================================
            while rclpy.ok():

                force = get_tool_force(DR_BASE)
                fz = force[2]

                current_pos = get_current_posx()
                z = current_pos[0][2]

                self.get_logger().info(f"현재 Z={z:.2f} mm, Fz={fz:.2f} N")

                # 여기서 직접 판단
                if abs(fz) >= force_threshold:
                    contacted = True

                    self.get_logger().info(f"바닥 접촉 감지: |Fz|={abs(fz):.2f} N")
                    break

                if time.time() - start_time > timeout:
                    self.get_logger().warning("접촉 감지 시간 초과")
                    break

                time.sleep(0.05)

        finally:
            release_force(time=0.2)
            time.sleep(0.2)

            release_compliance_ctrl()

            self.get_logger().info(
                "Force / Compliance OFF"
            )

        # 접촉 못 했으면 절대 열지 않기
        if not contacted:
            self.get_logger().warning("바닥 접촉이 없어서 그리퍼를 열지 않습니다.")
            error = String()
            error.data = "바닥 접촉이 없어서 그리퍼를 열지 않습니다."
            self.pub_error_status.publish(error)
            return False

        self.gripper.move_gripper(width_val=1000,force_val=200)

        time.sleep(1.0)

        self.get_logger().info("물체 내려놓기 완료")

        return True

    # 원뿔 스캔 실행
    def run_cone_scan(self, center_pose, target_name, max_retries=3):
        for attempt in range(max_retries):
            if target_name == "hand":
                coordinate = self.hand_scanner.scan(center_pose, target_name)
            else:
                coordinate = self.target_scanner.scan(center_pose, target_name)

            if coordinate is not None:
                self.detected_coordinate = coordinate
                self.get_logger().info(f"{target_name} 최종 좌표: {coordinate}")
                return True
            
            self.get_logger().warning(f"{target_name} 좌표 탐색 실패 (시도 {attempt + 1}/{max_retries})")
            
        self.get_logger().info(f"{target_name} 좌표 탐색 최종 실패")
        error = String()
        error.data = f"{target_name} 좌표 탐색 최종 실패"
        self.pub_error_status.publish(error)
        return False

    # ##############################################################################
    # table_scan 
    def run_table_scan(self, goal_name):
        """
        로봇을 두 개의 Waypoint로 순차 이동시킨다.

        각 Waypoint에 도착한 후 YOLO 노드에
        ScanRequest 서비스를 요청하고 응답을 기다린다.
        """
        self.get_logger().info("DB_SCAN_CASE 서비스 호출 시작")
        response = self.get_object_from_db(goal_name)

        if response is None:
            self.get_logger().error(f"{goal_name} DB 좌표 수신 실패")
            return

        waypoints = []

        for i in range(1, 4):
            field_name = f"pose_{i}"
            pose = getattr(response, field_name, None)

            if pose is None:
                self.get_logger().warning(
                    f"{field_name} 값이 None이므로 반복을 종료합니다."
                )
                break

            waypoints.append(list(pose))

        time.sleep(1.0)

        self.get_logger().info("DB_SCAN_CASE 서비스 호출 완료")

        self.get_logger().info("테이블 스캔 작업을 시작합니다.")

        try:
            for i, waypoint in enumerate(waypoints, start=1):

                move_waypoint = list(waypoint)
                move_waypoint[2] -= 250.0
                field_name = f"pose_name_{i}"
                waypoint_name = getattr(response, field_name, f"Waypoint_{i}")

                # 1. 로봇 이동
                move_success = self.move_to_position(waypoint_name, move_waypoint)

                if not move_success:
                    error = String()
                    error.data = f"{waypoint_name} 이동 실패"
                    self.pub_error_status.publish(error)
                    self.get_logger().error( f"{waypoint_name} 이동 실패로 테이블 스캔을 중단합니다.")
                    return

                # 2. YOLO 스캔 요청
                scan_success = self.request_yolo_scan(waypoint_name)

                if not scan_success:
                    error = String()
                    error.data = f"{waypoint_name} YOLO 스캔 실패"
                    self.pub_error_status.publish(error)
                    self.get_logger().error(f"{waypoint_name} YOLO 스캔 실패로 다음 Waypoint 이동을 중단합니다.")
                    return
                self.get_logger().info(f"{waypoint_name} 작업 완료")

            table_save_done = Bool()
            table_save_done.data = True
            self.pub_table_scan.publish(table_save_done)
            self.get_logger().info("모든 테이블 Waypoint 스캔이 완료되었습니다.")

        except Exception as error:
            self.get_logger().error(f"테이블 스캔 중 예외 발생: {error}")

    # table_scan 시 waypoint마다 yolo에게 스캔 요청 및 응답을 기다리는 함수
    def request_yolo_scan(self, waypoint_name):
        """
        YOLO 노드의 /yolo_scan_request 서비스를 호출한다.

        반환값:
            True:   서비스 통신 성공 및 response.success == True
            False:  서비스 없음, 응답 시간 초과, 통신 예외 또는 response.success == False
        """

        self.get_logger().info(f"YOLO 서비스 확인 중: {waypoint_name}")
        # 서비스 서버가 살아있는지 확인 => True/ False
        service_ready = self.scan_table_client.wait_for_service(timeout_sec=YOLO_SERVICE_WAIT_TIMEOUT)
        if not service_ready:
            self.get_logger().error("/yolo_scan_request 서비스를 찾을 수 없습니다.")
            return False

        request = ScanRequest.Request()
        request.waypoint_id = waypoint_name
        self.get_logger().info(f"YOLO 스캔 요청 전송: waypoint_id={waypoint_name}")
        response = self.call_service( self.scan_table_client, request, timeout=YOLO_SCAN_RESPONSE_TIMEOUT)

        if response is None:
            self.get_logger().error(f"YOLO 서비스 응답 수신 실패: {waypoint_name}")
            return False
        
        # YOLO가 전달한 정보는 현재 제어 판단에는 사용하지 않고
        # 로그만 출력한다.
        self.get_logger().info(
            f"YOLO 응답 수신: "
            f"waypoint_id={waypoint_name}, "
            f"success={response.success}, "
            f"detected_count={response.detected_count}, "
            f"message='{response.message}'"
        )

        if not response.success:
            self.get_logger().error(f"YOLO 스캔 자체가 실패했습니다: {response.message}")
            return False

        return True

    # 그립퍼 Adaptive Grip logic start
    def run_adaptive_grip(self, bbox_w, bbox_h, dist):
        pixel_area = bbox_w * bbox_h
        real_area_estimate = pixel_area * (dist ** 2)
        K = 0.00001
        
        force = min(int(150 + (real_area_estimate * K)), 400)
        
        self.get_logger().info(f"그리퍼 계산 [면적:{pixel_area} | 거리:{dist}m | 힘:{force}]")
        
        self.gripper.move_gripper(width_val=0, force_val=force)
        
        time.sleep(2.0)
        if self.gripper.get_status()[0] == 1:
            self.get_logger().info(">> 파지 성공!")
            return True
        
        self.get_logger().warn(">> 파지 실패")
        return False

    # 그립퍼 열기 서비스 콜백
    def ungrip_callback(self, request, response):            
        self.gripper.move_gripper(width_val=1000, force_val=200)
        msg = Bool()
        msg.data = True
        self.pub_task_completed.publish(msg)  # 최종 작업 완료
        response.success = True
        return response
    
    # Move to position using movel
    def move_to_position(self, move_name, target_position):

        self.get_logger().info(f"{move_name} ({target_position}) 이동 시작")
        try:
            movel(target_position, vel=VELOCITY, acc=ACCELERATION)
        except Exception as exc:
            self.report_error(f"{move_name} ({target_position}) 이동 중 오류 발생: {exc}, {move_name} 이동 실패")
            return False

        self.get_logger().info(f"{move_name} 이동 완료")
        return True

    # DB에서 좌표 가져오는 함수
    def get_object_from_db(self, target_data):
        """
        req_type에 따라 호출할 서비스를 결정합니다.
        - "fixed_pose": pos1, pos2, HAND_SCAN 등 6자유도 포즈 (GetFixedPose)
        - "scan_case": CASE_1 등 웨이포인트 2개 (GetScanCase)
        - "object": drink, mouse 등 3자유도 타겟 객체 (GetObjectCoordinate)
        """

        if target_data == "table_scan":
            request = GetScanCase.Request()
            request.case_name = "CASE_1"
            return self.call_service(self.db_scan_case_client, request)


        elif target_data in ("hand", "pos1", "pos2", "pos3"):
            request = GetFixedPose.Request()
            if target_data == "hand":
                request.pose_name = "HAND_SCAN"
            else:
                request.pose_name = target_data
            return self.call_service(self.db_fixed_pose_client, request)
           
        else: # 기본값은 object
            request = GetFixedPose.Request()
            request.pose_name = target_data
            return self.call_service(self.db_fixed_pose_client, request)
           
    def call_service(self, client, request, timeout=10.0):
        future = client.call_async(request)

        response_event = threading.Event()
        future.add_done_callback(lambda _: response_event.set())

        received = response_event.wait(timeout=timeout)

        if not received:
            self.report_error("서비스 응답 시간 초과")
            return None

        try:
            return future.result()

        except Exception as exc:
            self.report_error(f"서비스 호출 실패: {exc}")
            return None
        # if not received:
        #     self.get_logger().error("서비스 응답 시간 초과")
        #     error = String()
        #     error.data = "서비스 응답 시간 초과"
        #     self.pub_error_status.publish(error)
        #     return None

        # try:
        #     return future.result()

        # except Exception as error:
        #     log_message = f"서비스 호출 실패: {error}"
        #     self.report_error(log_message)
        #     self.return_to_home()
            # self.get_logger().error(f"서비스 호출 실패: {error}")
            # error = String()
            # error.data = f"서비스 호출 실패: {error}"
            # self.pub_error_status.publish(error)
            # return None

    def report_error(self, log_message, status_message=None):

        self.get_logger().error(log_message)
        msg = String()
        msg.data = (status_message if status_message is not None else log_message)
        self.pub_error_status.publish(msg)

    def return_to_home(self):
        try:
            movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACCELERATION)
            self.get_logger().info("대기모드 복귀 완료")

        except Exception as exc:
            self.report_error(f"대기모드 복귀 실패: {exc}")

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
        node.gripper.close_connection()
        node.destroy_node()
        dsr_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()