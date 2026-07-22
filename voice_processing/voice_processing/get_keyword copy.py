import os
import rclpy
import pyaudio
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate

# ★ 우리가 정의한 커스텀 메시지 import (Trigger 대신 사용)
from hey_doopal_msg.srv import VoiceKeyword
from voice_processing.MicController import MicController, MicConfig
from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")

############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("get_keyword_node")

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        # 프롬프트는 기존 그대로 유지
        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - airpods, cable, drink, mouse, pos1, pos2, pos3

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "airpods를 pos1에 가져다 놔"  
            출력: airpods / pos1

            - 입력: "왼쪽에 있는 airpods와 cable를 pos1에 넣어줘"  
            출력: airpods cable / pos1

            - 입력: "왼쪽에 있는 mouse를줘"  
            출력: mouse /

            - 입력: "왼쪽에 있는 drink를 줘"  
            출력: drink /

            - 입력: "airpods는 pos2에 두고 cable는 pos1에 둬"  
            출력: airpods cable / pos2 pos1

            <사용자 입력>
            "{user_input}"                
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)

        # 오디오 설정
        mic_config = MicConfig(
            chunk=3840,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=12,
            buffer_size=3840,
        )
        self.mic_controller = MicController(config=mic_config)
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        # 현재 노드의 모드 관리 ("WAKEUP": 최초 시동어 대기, "CONFIRM": "받았어" 대기)
        self.current_mode = "WAKEUP"

        # 1. 로봇 제어 노드로부터 모드 변경 명령을 받는 서비스 서버
        self.mode_srv = self.create_service(
            VoiceKeyword, "set_voice_mode_service", self.handle_mode_change
        )

        # 2. 분석된 키워드(target, goal) 또는 "받았어" 트리거를 로봇 제어 노드로 쏘기 위한 클라이언트
        self.client = self.create_client(VoiceKeyword, "voice_keyword_service")

        self.get_logger().info("GetKeyword Node initialized with Mode Control.")
        
        # 타이머를 통해 메인 루프 실행 (spin과 병행하기 위함)
        self.timer = self.create_timer(0.1, self.main_loop)

    def handle_mode_change(self, request, response):
        """로봇 제어 노드가 모드를 바꿀 때 호출되는 서비스"""
        self.current_mode = request.mode
        self.get_logger().info(f"★ 모드 변경됨 -> {self.current_mode} ★")
        response.success = True
        response.message = f"Mode changed to {self.current_mode}"
        return response

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        # 예외 처리 안전장치 추가
        if "/" not in result:
            return [], []

        obj_str, target_str = result.strip().split("/")
        objects = obj_str.split()
        targets = target_str.split()

        print(f"llm's response: {result}")
        print(f"object: {objects}")
        print(f"target: {targets}")
        return objects, targets

    def send_to_robot(self, target_val, goal_val):
        """로봇 제어 노드로 서비스 요청 전송"""
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 제어 서버 연결 대기 중...')

        request = VoiceKeyword.Request()
        request.mode = self.current_mode
        request.target = target_val
        request.goal = goal_val

        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"서버 응답 수신 성공: {response.message}")
        except Exception as e:
            self.get_logger().error(f"서비스 호출 실패: {e}")

    def main_loop(self):
        """모드에 따라 동작을 다르게 가져가는 메인 루프"""
        
        # 1. 최초 시동어("헤이 두팔") 대기 모드
        if self.current_mode == "WAKEUP":
            try:
                self.mic_controller.open_stream()
                self.wakeup_word.set_stream(self.mic_controller.stream)
            except OSError:
                return

            # 시동어가 감지될 때까지 대기
            if not self.wakeup_word.is_wakeup():
                return

            self.get_logger().info("시동어 감지! 음성 인식을 시작합니다.")
            
            # STT 및 키워드 추출
            output_message = self.stt.speech2text()
            objects, targets = self.extract_keyword(output_message)

            if objects and targets:
                target_str = objects[0] # 첫 번째 도구
                goal_str = targets[0]   # 첫 번째 목적지
                self.get_logger().warn(f"추출된 도구: {target_str}, 목적지: {goal_str}")

                # 로봇 제어 서버로 전송
                self.send_to_robot(target_str, goal_str)
                
                # 전송 직후 모드를 SLEEP으로 변경하여 중복 인식 방지 (로봇이 제어권을 가짐)
                self.current_mode = "SLEEP"

        # 2. 손 건네주기 상황에서 "받았어" 입력을 기다리는 모드
        elif self.current_mode == "CONFIRM":
            self.get_logger().info("손 위치 도달 완료. '받았어' 키워드 대기 중...")
            
            try:
                self.mic_controller.open_stream()
                self.wakeup_word.set_stream(self.mic_controller.stream)
            except OSError:
                return

            # 임시로 음성 입력을 받아 STT 수행 (또는 '받았어' 전용 wake_word 모델 활용 가능)
            output_message = self.stt.speech2text()
            self.get_logger().info(f"인식된 텍스트: {output_message}")

            # "받았어" 라는 단어가 포함되어 있는지 확인
            if "받았어" in output_message or "받" in output_message:
                self.get_logger().warn("'받았어' 키워드 검출! 로봇 제어 노드로 트리거 전송")
                
                # 로봇 제어 노드에 "받았어" 신호 전송 (예: target을 'ACK_RECEIVED'로 전달)
                self.send_to_robot("ACK_RECEIVED", "hand_done")
                
                # 전송 후 SLEEP 모드로 전환
                self.current_mode = "SLEEP"

        elif self.current_mode == "SLEEP":
            # 로봇이 동작하는 동안 음성 노드는 대기
            pass


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()