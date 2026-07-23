import time

import rclpy
from rclpy.action import (
    ActionServer,
    CancelResponse,
)
from rclpy.node import Node

from hey_doopal_msg.action import FindTargetOrder


class MockYoloServer(Node):

    def __init__(self):
        super().__init__("mock_yolo_server")

        self.action_server = ActionServer(
            self,
            FindTargetOrder,
            "/find_target_order",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            "Mock YOLO Action Server 시작: /find_target"
        )

    def cancel_callback(self, goal_handle):
        self.get_logger().info(
            "탐색 취소 요청 수신"
        )
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_name = goal_handle.request.target_name

        self.get_logger().info(
            f"탐색 요청 수신: {target_name}"
        )

        feedback = FindTargetOrder.Feedback()

        for index in range(10):
            if goal_handle.is_cancel_requested:
                result = FindTargetOrder.Result()
                result.found = False
                result.coordinate = [0.0] * 6
                result.message = "탐색 취소"

                goal_handle.canceled()
                return result

            feedback.state = f"searching {index + 1}/10"
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(feedback.state)

            time.sleep(0.3)

        result = FindTargetOrder.Result()
        result.found = True
        result.coordinate = [
            500.0,
            100.0,
            300.0,
            180.0,
            0.0,
            180.0,
        ]
        result.message = f"{target_name} 탐지 성공"

        goal_handle.succeed()

        self.get_logger().info(
            f"탐지 성공: {target_name}, "
            f"좌표={list(result.coordinate)}"
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    node = MockYoloServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()