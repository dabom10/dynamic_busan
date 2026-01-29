#!/usr/bin/env python3
"""
Shake Node - Action Server for motion control
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from bartender_interfaces.action import Motion


class ShakeController(Node):
    def __init__(self):
        super().__init__("shake_node")
        self.get_logger().info("Shake Node initialized")

        # Action Server 생성
        self._action_server = ActionServer(
            self,
            Motion,
            'shake/motion',  # Action 이름
            self.execute_callback
        )

        self.get_logger().info("Shake Action Server ready (shake/motion)")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Action 실행 콜백 (Goal 수신 시 호출됨)"""
        self.get_logger().info(f"Received goal: motion_id={goal_handle.request.motion_id}, "
                               f"name={goal_handle.request.motion_name}, "
                               f"duration={goal_handle.request.duration_ms}ms")

        # Goal 정보 추출
        motion_id = goal_handle.request.motion_id
        motion_name = goal_handle.request.motion_name
        duration_ms = goal_handle.request.duration_ms

        # Feedback 메시지 생성
        feedback_msg = Motion.Feedback()

        # 모션 실행 (시뮬레이션)
        start_time = time.time()
        total_steps = 10

        for i in range(total_steps):
            # 진행률 계산
            progress = int((i + 1) / total_steps * 100)

            # Feedback 발행
            feedback_msg.progress = progress
            feedback_msg.current_step = f"Executing {motion_name} step {i + 1}/{total_steps}"
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f"Progress: {progress}% - {feedback_msg.current_step}")

            # 실제 모션 시간 시뮬레이션
            time.sleep(duration_ms / 1000 / total_steps)

        # 완료 시간 계산
        elapsed_ms = int((time.time() - start_time) * 1000)

        # Goal 성공 처리
        goal_handle.succeed()

        # Result 반환
        result = Motion.Result()
        result.success = True
        result.message = f"Motion '{motion_name}' completed successfully"
        result.total_time_ms = elapsed_ms

        self.get_logger().info(f"Motion completed in {elapsed_ms}ms")

        return result


def main(args=None):
    rclpy.init(args=args)
    node = ShakeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
