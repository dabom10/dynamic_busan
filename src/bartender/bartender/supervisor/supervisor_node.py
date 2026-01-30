#!/usr/bin/env python3
"""
Supervisor Node - 여러 노드의 순차 실행 제어
각 노드별 ActionClient를 통해 순차적으로 모션 실행
stt_node의 /customer_name 토픽 수신 시 시퀀스 시작
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from bartender_interfaces.action import Motion


class SupervisorNode(Node):
    def __init__(self):
        super().__init__("supervisor_node")
        self.get_logger().info("Supervisor Node initialized")

        # Callback Group (비동기 처리용)
        self._cb_group = ReentrantCallbackGroup()

        # 각 노드별 ActionClient
        self._action_clients = {
            'recipe': ActionClient(self, Motion, 'recipe/motion', callback_group=self._cb_group),
            'shake': ActionClient(self, Motion, 'shake/motion', callback_group=self._cb_group),
        }

        # 실행할 모션 시퀀스 (client: 어떤 노드로 보낼지)
        self.motion_sequence = [
            {'client': 'recipe', 'name': 'make_drink'},
            {'client': 'shake', 'name': 'shake_it'},
        ]
        self.current_index = 0
        self.is_running = False  # 중복 실행 방지
        self.current_customer = None  # 현재 고객 이름

        # /customer_name 토픽 구독 (stt_node에서 발행)
        self.create_subscription(
            String,
            '/customer_name',
            self.on_customer_name,
            10,
            callback_group=self._cb_group
        )

        self.get_logger().info("Waiting for /customer_name topic...")

    def on_customer_name(self, msg):
        """고객 이름 수신 시 시퀀스 시작"""
        customer_name = msg.data.strip()

        # 중복 실행 방지
        if self.is_running:
            self.get_logger().warn(f"Already running! Ignoring: {customer_name}")
            return

        # 같은 고객 이름이면 무시 (stt_node가 주기적으로 발행하므로)
        if customer_name == self.current_customer:
            return

        self.current_customer = customer_name
        self.get_logger().info(f"=== Order received: {customer_name} ===")
        self.start_sequence()

    def start_sequence(self):
        """시퀀스 시작"""
        self.is_running = True
        self.current_index = 0

        self.get_logger().info("Waiting for Action Servers...")

        # 서버 연결 확인 (타임아웃 사용)
        for name, client in self._action_clients.items():
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"  - {name}/motion server not available!")
                self.reset_state()
                return
            self.get_logger().info(f"  - {name}/motion connected")

        self.get_logger().info("All servers connected! Starting sequence...")
        self.execute_next()

    def execute_next(self):
        """다음 모션 실행"""
        if self.current_index >= len(self.motion_sequence):
            self.get_logger().info(f"=== All motions completed for {self.current_customer}! ===")
            self.reset_state()
            return

        motion = self.motion_sequence[self.current_index]
        client_name = motion['client']
        client = self._action_clients[client_name]

        self.get_logger().info(
            f"[{self.current_index + 1}/{len(self.motion_sequence)}] "
            f"Sending to {client_name}: {motion['name']}"
        )

        # Goal 생성
        goal = Motion.Goal()
        goal.motion_name = motion['name']

        # Goal 전송
        send_goal_future = client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback
        )
        send_goal_future.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, future):
        """Goal 수락됨"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")

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

    def reset_state(self):
        """시퀀스 완료 후 상태 초기화"""
        self.is_running = False
        self.current_customer = None
        self.current_index = 0


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()

    # MultiThreadedExecutor 사용
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
