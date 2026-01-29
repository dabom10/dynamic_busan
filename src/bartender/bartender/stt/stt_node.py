#!/usr/bin/env python3
"""
STT Node - Speech to Text with Database Integration
Captures microphone input, converts to text, and saves to database
"""
import rclpy
from rclpy.node import Node
#import speech_recognition as sr
from datetime import datetime
from bartender.db.db_client import DBClient

from openai import OpenAI
import sounddevice as sd
import scipy.io.wavfile as wav
import tempfile
import os
from dotenv import load_dotenv
from konlpy.tag import Komoran

from std_msgs.msg import String


# .env 로드
# 절대 경로라서 수정 필요
load_dotenv(dotenv_path="/home/rokey/Tutorial_2026/Tutorial/VoiceProcessing/.env")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

class STTNode(Node):
    def __init__(self, api_key):
        super().__init__("stt_node")
        self.get_logger().info("STT Node initialized")

        # DB Client 초기화
        self.db_client = DBClient(self)
        '''
        # Speech Recognizer 초기화
        # self.recognizer = sr.Recognizer()
        # self.microphone = sr.Microphone()

        # # 마이크 노이즈 조정
        # with self.microphone as source:
        #     self.get_logger().info("Adjusting for ambient noise... Please wait")
        #     self.recognizer.adjust_for_ambient_noise(source, duration=2)
        #     self.get_logger().info("Ready for speech recognition")

        # # 타이머로 주기적으로 음성 인식 (5초마다)
        # 
        '''
        # 퍼블리시를 위한 변수 추가
        self.last_name = None

        # name 퍼블리셔 생성
        self.name_publisher = self.create_publisher(
            String,
            "/customer_name",
            10
        )
        # 퍼블리시 타이머
        self.name_publish_timer = self.create_timer(
            1.0,
            self.publish_name_periodic
        )



        # 클라이언트 타이머
        self.timer = self.create_timer(5.0, self.listen_and_process)

        self.client = OpenAI(api_key=api_key)
        self.duration = 5        # 녹음 시간 (초)
        self.samplerate = 16000  # Whisper 권장 샘플레이트

        # 노드 시작 시 쿼리 실행
        #self.query_logs_by_keyword()

    def publish_name_periodic(self):
        if self.last_name is None:
            return  # 아직 name이 없으면 아무 것도 안 함

        msg = String()
        msg.data = self.last_name
        self.name_publisher.publish(msg)
        self.get_logger().debug(f"Republished name: {self.last_name}")

    def listen_and_process(self):
        """마이크로 음성을 듣고 텍스트로 변환한 뒤 DB에 저장"""
        try:
            print("🎙️ 5초 동안 말해주세요...")

            # 1️⃣ 마이크로 음성 녹음
            audio = sd.rec(
                int(self.duration * self.samplerate),
                samplerate=self.samplerate,
                channels=1,
                dtype="int16",
            )
            sd.wait()
            print("✅ 녹음 완료, STT 처리 중...")

            # 2️⃣ 임시 wav 파일 생성
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
                wav.write(temp_wav.name, self.samplerate, audio)

                # 3️⃣ Whisper API로 STT
                with open(temp_wav.name, "rb") as f:
                    transcript = self.client.audio.transcriptions.create(
                        model="whisper-1",
                        file=f,
                    )
            line = transcript.text

            # [4] 문장에서 명사만 추출
            komoran = Komoran()
            nouns = komoran.nouns(line)

            stop_words = ['안녕', '이름', '잔']
            # 필요 없는 말들 제외 리스트

            filtered = [
                n for n in nouns
                if not any(word in n for word in stop_words)
            ]
            # 제외 리스트에 겹치는 말 거르기

            print("원본:", nouns)
            print("필터 후:", filtered)

            # 이름, 메뉴 부분 변수에 지정
            name = filtered[0]
            menu = " ".join(filtered[1:])

            print(name, menu)

            # DB에 저장
            self.save_to_database(name,menu)

            # 이름 퍼블리시 위해 변수 저장
            self.last_name = name

        except Exception as e:
            self.get_logger().error(f"Error in listen_and_process: {e}")
        '''
        try:
            self.get_logger().info("Listening...")

            with self.microphone as source:
                # 음성 입력 대기 (timeout=5초, phrase_time_limit=10초)
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)

            self.get_logger().info("Processing speech...")

            # Google Speech Recognition API 사용
            text = self.recognizer.recognize_google(audio, language='ko-KR')
            self.get_logger().info(f"Recognized text: {text}")

            # DB에 저장
            self.save_to_database(text)

        except sr.WaitTimeoutError:
            self.get_logger().debug("No speech detected (timeout)")
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results from Google Speech Recognition service; {e}")
        except Exception as e:
            self.get_logger().error(f"Error in listen_and_process: {e}")

        '''

    def save_to_database(self, text: str, text2: str):
        """인식된 텍스트를 데이터베이스에 저장"""
        #timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # INSERT 쿼리 작성
        query = f"""
        INSERT INTO bartender_order_history (name, menu )
        VALUES ('{text.replace("'", "''")}','{text2.replace("'", "''")}')
        """

        # DB에 쿼리 전송 및 응답 대기
        request_id = self.db_client.execute_query_with_response(
            query,
            callback=self.on_db_response
        )

        self.get_logger().info(f"Sent query to DB (request_id: {request_id})")

    def on_db_response(self, response_data: dict):
        """DB INSERT 쿼리 응답 처리"""
        if response_data.get('success'):
            self.get_logger().info("Successfully saved to database")
            self.get_logger().info(f"Response: {response_data}")

            # INSERT 성공 후 최근 로그 조회 (예시)
            self.query_recent_logs(limit=5)
        else:
            self.get_logger().error(f"Failed to save to database: {response_data.get('error')}")

    # ============ SELECT 쿼리 예시들 ============

    def query_recent_logs(self, limit: int = 10):
        """
        최근 N개의 STT 로그 조회 (parameter 예시)

        Args:
            limit: 조회할 개수
        """
        query = f"""
        SELECT id, text, created_at
        FROM stt_logs
        ORDER BY created_at DESC
        LIMIT {limit}
        """

        request_id = self.db_client.execute_query_with_response(
            query,
            callback=self.on_select_response
        )

        self.get_logger().info(f"Querying recent {limit} logs (request_id: {request_id})")

    def query_logs_by_keyword(self, keyword: str = '유성'):
        """
        특정 키워드를 포함하는 로그 조회 (LIKE 사용 예시)

        Args:
            keyword: 검색할 키워드
        """
        # SQL Injection 방지를 위해 single quote escape
        escaped_keyword = keyword.replace("'", "''")
        #escaped_keyword = '유성'
        query = f"""
        SELECT name, percent
        FROM bartender_order_history
        WHERE name LIKE '%{escaped_keyword}%'
        ORDER BY created_at DESC
        """

        request_id = self.db_client.execute_query_with_response(
            query,
            callback=self.on_select_response
        )

        self.get_logger().info(f"Querying logs with keyword '{keyword}' (request_id: {request_id})")

    def query_logs_by_date_range(self, start_date: str, end_date: str):
        """
        특정 날짜 범위의 로그 조회 (날짜 parameter 예시)

        Args:
            start_date: 시작 날짜 (YYYY-MM-DD HH:MM:SS)
            end_date: 종료 날짜 (YYYY-MM-DD HH:MM:SS)
        """
        query = f"""
        SELECT id, text, created_at
        FROM stt_logs
        WHERE created_at BETWEEN '{start_date}' AND '{end_date}'
        ORDER BY created_at DESC
        """

        request_id = self.db_client.execute_query_with_response(
            query,
            callback=self.on_select_response
        )

        self.get_logger().info(f"Querying logs from {start_date} to {end_date} (request_id: {request_id})")

    def on_select_response(self, response_data: dict):
        """DB SELECT 쿼리 응답 처리"""
        if response_data.get('success'):
            result = response_data.get('result', [])
            self.get_logger().info(f"Query successful! Retrieved {len(result)} rows")

            # 조회 결과 출력
            for row in result:
                self.get_logger().info(f"  - {row}")
                # 개별 필드 접근 예시
                # name = row.get('name')
                # percent = row.get('percent')
        else:
            self.get_logger().error(f"Query failed: {response_data.get('error')}")


def main(args=None):
    rclpy.init(args=args)
    node = STTNode(OPENAI_API_KEY)  # __init__에서 query_logs_by_keyword 자동 실행

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()