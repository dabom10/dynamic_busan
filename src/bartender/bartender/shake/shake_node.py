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
        motion_name = goal_handle.request.motion_name
        self.get_logger().info(f"Received goal: name={motion_name}")

        # Feedback 메시지 생성
        feedback_msg = Motion.Feedback()
        start_time = time.time()

        # 모션 리스트 정의 (step_name)
        # TODO: 실제 movel, movej 좌표로 교체
        motions = [
            ("Move to ready position"),
            ("Move to shake position"),
            ("Shaking motion 1"),
            ("Shaking motion 2"),
            ("Return to home"),
        ]

        total_motions = len(motions)

        for i, (step_name) in enumerate(motions):
            # 진행률 계산 (각 모션 완료 시)
            progress = int((i + 1) / total_motions * 100)

            # Feedback 발행 (모션 시작)
            #feedback_msg.progress = progress - (100 // total_motions)
            #feedback_msg.current_step = f"[{i + 1}/{total_motions}] {step_name}"
            #goal_handle.publish_feedback(feedback_msg)
            #self.get_logger().info(f"Starting: {feedback_msg.current_step}")

            # 실제 모션 실행
            # TODO: 실제 로봇 제어 코드로 교체
            # self.movel(position) 또는 self.movej(position)
            if step_name == 'Move to ready position':
                pass
            elif step_name == 'Shaking motion 1':
                pass

            time.sleep(0.5)  # 시뮬레이션

            # Feedback 발행 (모션 완료)
            feedback_msg.progress = progress
            feedback_msg.current_step = f"[{i + 1}/{total_motions}] {step_name} - Done"
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Completed: {step_name} ({progress}%)")

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
