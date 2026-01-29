#!/usr/bin/env python3
"""
Supervisor Node - 여러 노드의 순차 실행 제어
각 노드별 ActionClient를 통해 순차적으로 모션 실행
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from bartender_interfaces.action import Motion


class SupervisorNode(Node):
    def __init__(self):
        super().__init__("supervisor_node")
        self.get_logger().info("Supervisor Node initialized")

        # 각 노드별 ActionClient
        self._clients = {
            'recipe': ActionClient(self, Motion, 'recipe/motion'),
            'shake': ActionClient(self, Motion, 'shake/motion'),
        }

        # 실행할 모션 시퀀스 (client: 어떤 노드로 보낼지)
        self.motion_sequence = [
            {'client': 'recipe', 'id': 1, 'name': 'make_drink', 'duration_ms': 2000},
            {'client': 'shake', 'id': 2, 'name': 'shake_it', 'duration_ms': 3000},
        ]
        self.current_index = 0

        # 모든 서버 연결 대기
        self.get_logger().info("Waiting for Action Servers...")
        for name, client in self._clients.items():
            client.wait_for_server()
            self.get_logger().info(f"  - {name}/motion connected")

        self.get_logger().info("All servers connected! Starting sequence...")
        self.execute_next()

    def execute_next(self):
        """다음 모션 실행"""
        if self.current_index >= len(self.motion_sequence):
            self.get_logger().info("=== All motions completed! ===")
            return

        motion = self.motion_sequence[self.current_index]
        client_name = motion['client']
        client = self._clients[client_name]

        self.get_logger().info(
            f"[{self.current_index + 1}/{len(self.motion_sequence)}] "
            f"Sending to {client_name}: {motion['name']}"
        )

        # Goal 생성
        goal = Motion.Goal()
        goal.motion_id = motion['id']
        goal.motion_name = motion['name']
        goal.duration_ms = motion['duration_ms']

        # Goal 전송
        future = client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, future):
        """Goal 수락됨"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        """진행 상황"""
        fb = feedback_msg.feedback
        self.get_logger().info(f"  Progress: {fb.progress}% - {fb.current_step}")

    def on_result(self, future):
        """완료 → 다음 실행"""
        result = future.result().result
        self.get_logger().info(f"  Completed: {result.message} ({result.total_time_ms}ms)")

        self.current_index += 1
        self.execute_next()


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
