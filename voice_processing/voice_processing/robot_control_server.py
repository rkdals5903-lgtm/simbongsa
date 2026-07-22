import rclpy
from rclpy.node import Node
from hey_doopal_msg.srv import VoiceKeyword
import time

class RobotControlServer(Node):
    def __init__(self):
        super().__init__('robot_control_server')
        
        # 1. 음성 노드로부터 최초 명령을 받는 서버 (입구)
        self.srv = self.create_service(VoiceKeyword, 'voice_keyword_service', self.handle_voice_command)
        
        # 2. 음성 노드의 모드를 바꾸기 위한 클라이언트 (출구)
        self.mode_client = self.create_client(VoiceKeyword, 'set_voice_mode_service')
        
        self.get_logger().info('제어 서버 대기 중...')

    def change_voice_mode(self, mode):
        """음성 노드에 모드 변경 명령을 쏘는 함수"""
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('음성 모드 서비스 연결 대기 중...')
            
        req = VoiceKeyword.Request()
        req.mode = mode
        future = self.mode_client.call_async(req)
        # 필요하다면 future 콜백 처리 추가

    def handle_voice_command(self, request, response):
        self.get_logger().info(f'★ 음성 명령 수신 완료! ★')
        self.get_logger().info(f'-> 대상: {request.target}')
        self.get_logger().info(f'-> 위치: {request.goal}')
        
        # 명령을 받았으니, 이제 음성 노드는 시동어 듣지 말고 멈춰있도록 SLEEP 또는 이동 모드로 전환 명령
        self.change_voice_mode("SLEEP")

        # [여기에 로봇 제어 로직 시뮬레이션]
        if request.goal == "hand":
            self.get_logger().info('로봇: 손 위치로 이동 및 5cm 접근 대기 중...')
            # 예: 5cm 거리 감지 루프 가정...
            time.sleep(2) 
            self.get_logger().info('로봇: 5cm 거리 도달! Stop 및 "받았어" 대기 모드로 전환 요청')
            
            # 음성 노드에 'CONFIRM' 모드(받았어 키워드 감지)로 전환 명령!
            self.change_voice_mode("CONFIRM")
            
        else:
            self.get_logger().info(f'로봇: {request.goal}로 이동 및 물건 배치 중...')
            time.sleep(3)
            self.get_logger().info('로봇: 원위치 복귀 완료. 다시 시동어 대기 모드로 복귀.')
            
            # 다시 시동어 대기 모드로 전환
            self.change_voice_mode("WAKEUP")

        response.success = True
        response.message = "Command processed successfully"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()