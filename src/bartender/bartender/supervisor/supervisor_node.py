#!/usr/bin/env python3
"""
Supervisor Node - STT + 모션 시퀀스 통합 (Topping 없는 버전)

흐름: Wakeup 감지 → STT → DB 저장 → tracking에 고객 이름 전달 → 모션 시퀀스 실행
시퀀스: recipe → shake

연동 토픽:
  - /customer_name (pub): tracking_node에 고객 이름 전달
  - /manufacturing_done (pub): recovery_node에 제작 완료 신호

"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from pathlib import Path
from std_msgs.msg import String

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
from difflib import get_close_matches

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
        self.current_menu = None  # 현재 주문 메뉴 (cup_pick에 전달)

        # 유효한 메뉴 목록 (recipe.json의 recipe_id = DB의 menu_seq)
        self.valid_menus = [
            "블루 사파이어", "블루사파이어",
            "테킬라 선라이즈", "테킬라선라이즈",
            "퍼플 레인", "퍼플레인",
            "진 앤 토닉", "진앤토닉",
            "트로피컬 오션", "트로피컬오션",
            "화이트 마가리타", "화이트마가리타",
            "블루 라군", "블루라군",
        ]

        # DB Client
        self.db_client = DBClient(self)

        # Publishers (tracking, recovery 연동)
        self.pub_customer_name = self.create_publisher(String, '/customer_name', 10)
        self.pub_manufacturing_done = self.create_publisher(String, '/manufacturing_done', 10)

        # OpenAI
        self.openai_client = OpenAI(api_key=api_key)
        self.duration = 5
        self.samplerate = 16000

        # 확인 단계 설정 (False로 바꾸면 확인 단계 생략)
        self.enable_confirmation = True
        self.confirmation_duration = 5  # 확인 응답 대기 시간 (초)

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

            # 메뉴를 먼저 찾고, 그 이전을 이름으로 처리
            name_parts = []
            menu_parts = []

            for noun in filtered:
                # 현재 명사가 메뉴에 포함되는지 확인
                is_menu = False
                for valid_menu in self.valid_menus:
                    if noun in valid_menu.replace(" ", ""):
                        is_menu = True
                        break

                if is_menu:
                    menu_parts.append(noun)
                else:
                    # 메뉴가 아직 안 나왔으면 이름에 추가
                    if not menu_parts:
                        name_parts.append(noun)

            name = "".join(name_parts)  # 공백 없이 결합 (예: "서동" + "찬" = "서동찬")
            menu = " ".join(menu_parts)  # 공백으로 결합 (예: "블루 사파이어")

            # 이름 저장 및 tracking에 전달
            self.current_customer = name
            name_msg = String()
            name_msg.data = name
            self.pub_customer_name.publish(name_msg)
            self.get_logger().info(f"[PUB] /customer_name: {name}")

            # 메뉴가 없으면 메뉴만 다시 받기
            if not menu:
                self.get_logger().warn(f"이름 '{name}'은(는) 확인되었습니다. 메뉴를 말해주세요.")
                self.get_logger().info(f"📋 가능한 메뉴: {', '.join([m for m in self.valid_menus if ' ' in m])}")
                self.listen_for_menu_only()
                return

            # 메뉴 검증
            valid_menu = self.validate_menu(menu)
            if valid_menu:
                # 확인 단계 (enable_confirmation이 True일 때만)
                if self.enable_confirmation:
                    if not self.ask_confirmation(name, valid_menu):
                        self.get_logger().warn("❌ 다시 입력해주세요.")
                        self.listen_and_process()
                        return

                self.current_menu = valid_menu
                self.save_to_database(name, valid_menu)
                self.get_logger().info(f"=== Order: {name}, Menu: {valid_menu} ===")
                self.start_sequence()
            else:
                self.get_logger().warn(f"❌ '{menu}'은(는) 잘못된 메뉴입니다. 다시 말해주세요.")
                self.get_logger().info(f"📋 가능한 메뉴: {', '.join([m for m in self.valid_menus if ' ' in m])}")
                self.listen_for_menu_only()

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

    def validate_menu(self, menu: str) -> str:
        """메뉴 유효성 검사. 유효하면 정규화된 메뉴명 반환, 아니면 None"""
        menu_normalized = menu.replace(" ", "")  # 공백 제거하여 비교

        # 1. 정확히 일치하는지 확인
        for valid_menu in self.valid_menus:
            valid_normalized = valid_menu.replace(" ", "")
            if menu_normalized == valid_normalized:
                # 공백 있는 정규 메뉴명 반환 (DB와 일치)
                if " " in valid_menu:
                    return valid_menu
                # 공백 없는 버전이면 공백 있는 버전 찾기
                for vm in self.valid_menus:
                    if vm.replace(" ", "") == valid_normalized and " " in vm:
                        return vm
                return valid_menu

        # 2. Fuzzy Matching (공백 있는 정규 메뉴만 대상)
        valid_menus_spaced = [m for m in self.valid_menus if " " in m]
        matches = get_close_matches(menu, valid_menus_spaced, n=1, cutoff=0.6)
        if matches:
            self.get_logger().info(f"🔍 Fuzzy match: '{menu}' → '{matches[0]}'")
            return matches[0]

        return None

    def ask_confirmation(self, name: str, menu: str) -> bool:
        """주문 확인 (예/아니요 판단)"""
        try:
            self.get_logger().info(f"고객님 성함은 '{name}', 메뉴는 '{menu}' 맞으신가요?")
            self.get_logger().info(f"({self.confirmation_duration}초 안에 대답해주세요)")

            # 음성 녹음
            audio = sd.rec(
                int(self.confirmation_duration * self.samplerate),
                samplerate=self.samplerate,
                channels=1,
                dtype="int16",
            )
            sd.wait()
            self.get_logger().info("확인 응답 처리 중...")

            # STT 처리
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
                wav.write(temp_wav.name, self.samplerate, audio)
                with open(temp_wav.name, "rb") as f:
                    transcript = self.openai_client.audio.transcriptions.create(
                        model="whisper-1",
                        file=f,
                    )

            response = transcript.text.lower()
            self.get_logger().info(f"확인 응답: {response}")

            # 긍정 단어 확인
            positive_words = ["예", "네", "맞", "응", "어", "yes", "ok", "오케이", "확인"]
            is_positive = any(word in response for word in positive_words)

            if is_positive:
                self.get_logger().info("✅ 주문이 확인되었습니다!")
                return True
            else:
                self.get_logger().warn("❌ 주문이 취소되었습니다.")
                return False

        except Exception as e:
            self.get_logger().error(f"확인 단계 에러: {e}")
            return False

    def listen_for_menu_only(self):
        """메뉴만 다시 입력받기 (이름은 유지)"""
        try:
            self.get_logger().info("메뉴를 다시 말해주세요 (5초)...")

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

            # 명사 추출 (메뉴만)
            komoran = Komoran()
            nouns = komoran.nouns(line)
            stop_words = ['안녕', '이름', '잔', '메뉴', '주문']
            filtered = [n for n in nouns if not any(word in n for word in stop_words)]

            self.get_logger().info(f"명사: {nouns} → 필터: {filtered}")

            if not filtered:
                self.get_logger().warn("메뉴 인식 실패. 처음부터 다시 시도해주세요.")
                self.reset_state()
                return

            menu = " ".join(filtered)

            # 메뉴 검증
            valid_menu = self.validate_menu(menu)
            if valid_menu:
                # 확인 단계 (enable_confirmation이 True일 때만)
                if self.enable_confirmation:
                    if not self.ask_confirmation(self.current_customer, valid_menu):
                        self.get_logger().warn("❌ 다시 입력해주세요.")
                        self.listen_for_menu_only()
                        return

                self.current_menu = valid_menu
                self.get_logger().info(f"=== 메뉴 확인: {valid_menu} ===")
                self.start_sequence()
            else:
                self.get_logger().warn(f"❌ '{menu}'은(는) 잘못된 메뉴입니다. 다시 말해주세요.")
                self.get_logger().info(f"📋 가능한 메뉴: {', '.join([m for m in self.valid_menus if ' ' in m])}")
                # 재귀적으로 메뉴만 다시 받기
                self.listen_for_menu_only()

        except Exception as e:
            self.get_logger().error(f"STT Error: {e}")
            self.reset_state()

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

            # recovery_node에 제작 완료 신호 전달
            done_msg = String()
            done_msg.data = self.current_customer if self.current_customer else ""
            self.pub_manufacturing_done.publish(done_msg)
            self.get_logger().info(f"[PUB] /manufacturing_done: {done_msg.data}")

            self.reset_state()
            return

        motion = self.motion_sequence[self.current_index]
        client = self._action_clients[motion['client']]

        # recipe 액션일 때는 실제 메뉴명 전달
        if motion['client'] == 'recipe' and self.current_menu:
            action_name = self.current_menu
        else:
            action_name = motion['name']

        self.get_logger().info(
            f"[{self.current_index + 1}/{len(self.motion_sequence)}] {motion['client']}: {action_name}"
        )

        goal = Motion.Goal()
        goal.motion_name = action_name
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
        self.current_menu = None
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
