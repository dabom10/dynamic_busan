#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
recovery_node_v2.py - 스레드 기반 복구 시스템

v1 대비 변경사항:
- ROS2 콜백 데드락 문제 해결
- 별도 스레드에서 spin_once() 실행
- 콜백에서는 플래그만 설정, 메인 스레드에서 로봇 동작 실행

문제 원인:
- rclpy.spin()이 단일 스레드로 실행
- 콜백 안에서 movej() 같은 blocking 함수 호출 시 데드락 발생
- movej()는 ROS2 서비스 응답을 기다리는데, spin()이 콜백 완료를 기다리므로 교착상태

해결 방법:
- spin_once()를 별도 스레드에서 실행
- 콜백에서는 플래그만 설정
- 메인 루프에서 플래그 확인 후 로봇 동작 실행
"""

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import io

# 한글 출력 설정
sys.stdout = io.TextIOWrapper(sys.stdout.detach(), encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.detach(), encoding='utf-8')

# 두산 라이브러리 초기화 모듈
import DR_init

# ========================================
# 로봇 설정 파라미터
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELJ = 60
ACCJ = 60
VELX, ACCX = 150, 150
J_READY = [0, 0, 90, 0, 90, 0]


class FailureRecoveryBot(Node):
    def __init__(self):
        super().__init__("failure_recovery_bot", namespace=ROBOT_ID)

        # 미션 상태 관리
        self.last_failed_customer = "미확인 고객"
        self.current_customer = None
        self.is_mission_running = False
        self.is_running = True  # 메인 루프 제어

        # 스레드 동기화용 Lock
        self._lock = threading.Lock()

        # 미션 요청 플래그 (콜백에서 설정, 메인 루프에서 처리)
        self.mission_requested = False
        self.mission_customer_name = None

        # 보관대 좌표 (실제 환경에 맞게 수정 필요)
        self.storage_posx = [400.0, 200.0, 300.0, 0.0, 180.0, 0.0]

        # 구독자 설정
        self.sub_disappeared = self.create_subscription(
            String, '/disappeared_customer_name', self.disappeared_cb, 10)
        self.sub_manufacturing = self.create_subscription(
            String, '/manufacturing_done', self.start_mission_cb, 10)

        # ROS2 spin 스레드 시작
        self.spin_thread = threading.Thread(target=self._spin_thread, daemon=True)
        self.spin_thread.start()

        self.get_logger().info('='*50)
        self.get_logger().info(f"M0609 복구 시스템 가동 (ID: {ROBOT_ID})")
        self.get_logger().info(f"토픽 구독:")
        self.get_logger().info(f"   - /disappeared_customer_name")
        self.get_logger().info(f"   - /manufacturing_done")
        self.get_logger().info('='*50)

    def _spin_thread(self):
        """별도 스레드에서 ROS2 콜백 처리"""
        while self.is_running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def disappeared_cb(self, msg):
        """인식 실패 고객 정보 수신"""
        with self._lock:
            self.last_failed_customer = msg.data.strip()
        self.get_logger().warn(f"인식 실패 접수: [{self.last_failed_customer}]")

    def start_mission_cb(self, msg):
        """
        제조 완료 신호 수신 시 미션 요청 플래그 설정

        주의: 콜백에서 직접 로봇 동작을 실행하면 데드락 발생!
        플래그만 설정하고 메인 루프에서 처리하도록 함
        """
        with self._lock:
            if self.is_mission_running:
                self.get_logger().warn("이미 미션 실행 중입니다. 무시합니다.")
                return

            # 미션 요청 플래그 설정 (실제 실행은 메인 루프에서)
            msg_data = msg.data.strip()
            self.mission_customer_name = msg_data if msg_data else self.last_failed_customer
            self.mission_requested = True

        self.get_logger().info(f"[미션 요청] {self.mission_customer_name}님의 음료 이동 요청됨")

    def recovery_sequence(self):
        """
        실제 로봇 동작 시퀀스

        이 함수는 메인 스레드에서 실행되어야 함 (콜백 X)
        """
        from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_MV_MOD_REL

        try:
            self.get_logger().info("=" * 40)
            self.get_logger().info("1. 홈 위치로 이동")
            movej(J_READY, vel=VELJ, acc=ACCJ)
            self.get_logger().info("   홈 위치 도착")
            wait(0.5)

            self.get_logger().info("2. 음료 파지 (Grip)")
            set_digital_output(1, 1)
            wait(1.0)

            self.get_logger().info("3. 보관대 상공으로 이동")
            target_up = list(self.storage_posx)
            target_up[2] += 100.0
            self.get_logger().info(f"   상공 좌표: {target_up}")
            movel(target_up, vel=VELX, acc=ACCX)
            wait(0.5)

            self.get_logger().info("4. 보관대에 내려놓기")
            self.get_logger().info(f"   목표 좌표: {self.storage_posx}")
            movel(self.storage_posx, vel=VELX//2, acc=ACCX//2)
            wait(0.5)

            self.get_logger().info("5. 그리퍼 해제")
            set_digital_output(1, 0)
            wait(1.0)

            self.get_logger().info("6. 안전 거리 확보 후 복귀")
            movel([0, 0, 100, 0, 0, 0], vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            wait(0.5)
            movej(J_READY, vel=VELJ, acc=ACCJ)

            self.get_logger().info("=" * 40)
            self.get_logger().info("시퀀스 완료")

        except Exception as e:
            self.get_logger().error(f"시퀀스 실행 중 오류: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            raise

    def run(self):
        """
        메인 실행 루프

        콜백에서 설정된 플래그를 확인하고 로봇 동작 실행
        """
        self.get_logger().info("메인 루프 시작. 미션 대기 중...")

        while self.is_running and rclpy.ok():
            # 미션 요청 확인
            with self._lock:
                if self.mission_requested and not self.is_mission_running:
                    self.mission_requested = False
                    self.is_mission_running = True
                    customer = self.mission_customer_name
                else:
                    customer = None

            # 미션 실행 (Lock 해제 후 실행 - 오래 걸리는 작업)
            if customer:
                self.get_logger().info('='*50)
                self.get_logger().info(f"[미션 시작] {customer}님의 음료 이동")
                self.get_logger().info(f"목표 좌표: {self.storage_posx}")
                self.get_logger().info('='*50)

                try:
                    self.recovery_sequence()
                    self.get_logger().info(f"[{customer}] 미션 완료")
                except Exception as e:
                    self.get_logger().error(f"동작 중 에러: {e}")
                finally:
                    with self._lock:
                        self.is_mission_running = False
                        self.current_customer = None

            time.sleep(0.1)  # CPU 부하 방지

    def shutdown(self):
        """종료 처리"""
        self.is_running = False
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)

    # 두산 라이브러리 기초 정보 등록
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    # 노드 생성
    node = FailureRecoveryBot()

    # 생성된 노드 객체를 라이브러리에 전달
    DR_init.__dsr__node = node

    # DSR_ROBOT2 함수 임포트 (노드 등록 후)
    from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_MV_MOD_REL

    try:
        node.get_logger().info("로봇 서비스 연결 확인 중...")
        node.get_logger().info("준비 완료. 토픽 대기 중...")

        # 메인 루프 실행 (rclpy.spin() 대신)
        node.run()

    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 종료됨")
    except Exception as e:
        node.get_logger().error(f"예상치 못한 오류: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
