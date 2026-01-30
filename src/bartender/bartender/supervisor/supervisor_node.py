#!/usr/bin/env python3
"""
Supervisor Node - STT + 모션 시퀀스 통합
Wakeup 감지 → STT → DB 저장 → 모션 시퀀스 실행
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from pathlib import Path

from bartender_interfaces.action import Motion
from bartender.db.db_client import DBClient

# 음성 인식
from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile
import os
from dotenv import load_dotenv
from konlpy.tag import Komoran

# wakeup
from bartender.stt.wakeup import WakeupWord
from bartender.stt import MicController

# .env 로드
env_path = Path.home() / 'dynamic_busan' / '.env'
load_dotenv(dotenv_path=env_path)
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")


class SupervisorNode(Node):
    def __init__(self, api_key):
        super().__init__("supervisor_node")
        self.get_logger().info("Supervisor Node initialized")

        # Callback Group
        self._cb_group = ReentrantCallbackGroup()

        # ActionClient
        self._action_clients = {
            'recipe': ActionClient(self, Motion, 'recipe/motion', callback_group=self._cb_group),
            'shake': ActionClient(self, Motion, '/dsr01/shake/motion', callback_group=self._cb_group),
        }

        # 모션 시퀀스
        self.motion_sequence = [
            {'client': 'recipe', 'name': 'make_drink'},
            {'client': 'shake', 'name': 'shake_it'},
        ]
        self.current_index = 0
        self.is_running = False
        self.current_customer = None

        # DB Client
        self.db_client = DBClient(self)

        # OpenAI
        self.openai_client = OpenAI(api_key=api_key)
        self.duration = 5
        self.samplerate = 16000

        # Wakeup
        self.mic = MicController.MicController()
        self.mic.open_stream()
        self.wakeup = WakeupWord(self.mic.config.buffer_size)
        self.wakeup.set_stream(self.mic.stream)

        # Timer
        self.wakeup_timer = self.create_timer(0.5, self.check_wakeup)
        self.get_logger().info("Ready - Waiting for wakeup word...")

    def check_wakeup(self):
        """Wakeup 감지"""
        if self.is_running:
            return

        if self.wakeup.is_wakeup():
            self.get_logger().info("Wakeup detected!")
            self.is_running = True
            self.listen_and_process()

    def listen_and_process(self):
        """STT 처리"""
        try:
            self.get_logger().info("5초 동안 말해주세요...")

            audio = sd.rec(
                int(self.duration * self.samplerate),
                samplerate=self.samplerate,
                channels=1,
                dtype="int16",
            )
            sd.wait()
            self.get_logger().info("녹음 완료, STT 처리 중...")

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
                wav.write(temp_wav.name, self.samplerate, audio)
                with open(temp_wav.name, "rb") as f:
                    transcript = self.openai_client.audio.transcriptions.create(
                        model="whisper-1",
                        file=f,
                    )

            line = transcript.text
            self.get_logger().info(f"STT 결과: {line}")

            # 명사 추출
            komoran = Komoran()
            nouns = komoran.nouns(line)
            stop_words = ['안녕', '이름', '잔']
            filtered = [n for n in nouns if not any(word in n for word in stop_words)]

            self.get_logger().info(f"명사: {nouns} → 필터: {filtered}")

            if not filtered:
                self.get_logger().warn("이름 인식 실패. 다시 시도해주세요.")
                self.is_running = False
                return

            name = filtered[0]
            menu = " ".join(filtered[1:]) if len(filtered) > 1 else ""

            self.save_to_database(name, menu)
            self.current_customer = name
            self.get_logger().info(f"=== Order: {name} ===")
            #sd.wait()
            self.start_sequence()

        except Exception as e:
            self.get_logger().error(f"STT Error: {e}")
            self.is_running = False

    def save_to_database(self, name: str, menu: str):
        """DB 저장"""
        query = f"""
        INSERT INTO bartender_order_history (name, menu)
        VALUES ('{name.replace("'", "''")}', '{menu.replace("'", "''")}')
        """
        self.db_client.execute_query_with_response(query)

    def start_sequence(self):
        """모션 시퀀스 시작"""
        self.current_index = 0
        self.get_logger().info("Connecting to Action Servers...")

        for name, client in self._action_clients.items():
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"{name}/motion not available!")
                self.reset_state()
                return
            self.get_logger().info(f"  {name}/motion connected")

        self.get_logger().info("Starting sequence...")
        self.execute_next()

    def execute_next(self):
        """다음 모션 실행"""
        if self.current_index >= len(self.motion_sequence):
            self.get_logger().info(f"=== Completed for {self.current_customer}! ===")
            self.reset_state()
            return

        motion = self.motion_sequence[self.current_index]
        client = self._action_clients[motion['client']]

        self.get_logger().info(
            f"[{self.current_index + 1}/{len(self.motion_sequence)}] {motion['client']}: {motion['name']}"
        )

        goal = Motion.Goal()
        goal.motion_name = motion['name']
        future = client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, future):
        """Goal 수락"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        """Feedback"""
        fb = feedback_msg.feedback
        self.get_logger().info(f"  {fb.progress}% - {fb.current_step}")

    def on_result(self, future):
        """Result → 다음 실행"""
        result = future.result().result
        self.get_logger().info(f"  Done: {result.message}")
        self.current_index += 1
        self.execute_next()

    def reset_state(self):
        """상태 초기화"""
        self.is_running = False
        self.current_customer = None
        self.current_index = 0
        self.get_logger().info("Ready for next customer...")


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode(OPENAI_API_KEY)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
