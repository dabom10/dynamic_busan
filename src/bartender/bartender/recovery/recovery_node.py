#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import io
import os

# [1. 기초 설정] 한글 출력 및 경로 설정
sys.stdout = io.TextIOWrapper(sys.stdout.detach(), encoding='utf-8')
sys.stderr = io.TextIOWrapper(sys.stderr.detach(), encoding='utf-8')

# [2. 두산 라이브러리 초기화 모듈 임포트]
import DR_init

# ========================================
# 로봇 설정 파라미터
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELJ = 60
ACCJ = 60
# VELJ, ACCJ = 60, 60     # 관절 속도/가속도
VELX, ACCX = 150, 150   # 직선 속도/가속도
J_READY = [0, 0, 0, 0, 0, 0] # 대기 위치

class FailureRecoveryBot(Node):
    def __init__(self):
        super().__init__("failure_recovery_bot", namespace=ROBOT_ID)
        
        # 미션 상태 관리
        self.last_failed_customer = "미확인 고객"
        self.current_customer = None
        self.is_mission_running = False  # 중복 실행 방지
        
        # 보관대 좌표 (x, y, z, a, b, c) 
        # ※ 실제 로봇 환경의 티칭 좌표로 반드시 수정 필요
        self.storage_posx = [400.0, 200.0, 300.0, 0.0, 180.0, 0.0]

        # 구독자 설정
        self.sub_disappeared = self.create_subscription(
            String, '/disappeared_customer_name', self.disappeared_cb, 10)
        self.sub_manufacturing = self.create_subscription(
            String, '/manufacturing_done', self.start_mission_cb, 10)

        self.get_logger().info('='*50)
        self.get_logger().info(f"🚀 M0609 복구 시스템 가동 (ID: {ROBOT_ID})")
        self.get_logger().info(f"📡 토픽 구독:")
        self.get_logger().info(f"   - /disappeared_customer_name")
        self.get_logger().info(f"   - /manufacturing_done")
        self.get_logger().info('='*50)

    def disappeared_cb(self, msg):
        """인식 실패 고객 정보 수신"""
        self.last_failed_customer = msg.data.strip()
        self.get_logger().warn(f"⚠️ 인식 실패 접수: [{self.last_failed_customer}]")
        self.get_logger().info(f"현재 저장된 실패 고객: {self.last_failed_customer}")

    def start_mission_cb(self, msg):
        """제조 완료 신호 수신 시 미션 시작"""
        if self.is_mission_running:
            self.get_logger().warn("⚠️ 이미 미션 실행 중입니다. 무시합니다.")
            return
        
        # 제조 완료 메시지에서 고객 이름 추출
        msg_data = msg.data.strip()
        self.current_customer = msg_data if msg_data else self.last_failed_customer
        
        self.get_logger().error('='*50)
        self.get_logger().error(f"🚨 [미션 시작] {self.current_customer}님의 음료 이동")
        self.get_logger().error(f"📍 목표 좌표: {self.storage_posx}")
        self.get_logger().error('='*50)
        
        # 로봇 동작 시퀀스 실행
        self.is_mission_running = True
        try:
            self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            # movej([0, 0, 90, 0, 100, 0], vel=VELJ, acc=ACCJ)
            self.recovery_sequence()
            self.get_logger().info(f"✅ [{self.current_customer}] 미션 완료")
        except Exception as e:
            self.get_logger().error(f"❌ 동작 중 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.is_mission_running = False
            self.current_customer = None

    def recovery_sequence(self):
        # from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_MV_MOD_REL
        """실제 로봇 동작 시퀀스 (DSR_ROBOT2 함수 사용)"""
        try:
            self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self.get_logger().info("1️⃣ 홈 위치로 이동")
            self.get_logger().info("1홈 위치로 이동")
            movej(J_READY, vel=VELJ, acc=ACCJ)
            self.get_logger().info("홈 위치로 이동")
            wait(0.5)
            self.get_logger().info("위치로 이동")
            
            self.get_logger().info("2️⃣ 음료 파지 (Grip)")
            set_digital_output(1, 1)  # 그리퍼 ON (예시 핀 1번)
            wait(1.0)

            self.get_logger().info("3️⃣ 보관대 상공으로 이동")
            target_up = list(self.storage_posx)  # 복사본 생성
            target_up[2] += 100.0  # Z축 위로 100mm
            self.get_logger().info(f"   상공 좌표: {target_up}")
            movel(target_up, vel=VELX, acc=ACCX)
            wait(0.5)

            self.get_logger().info("4️⃣ 보관대에 내려놓기")
            self.get_logger().info(f"   목표 좌표: {self.storage_posx}")
            movel(self.storage_posx, vel=VELX//2, acc=ACCX//2)
            wait(0.5)
            
            self.get_logger().info("5️⃣ 그리퍼 해제")
            set_digital_output(1, 0)  # 그리퍼 OFF
            wait(1.0)

            self.get_logger().info("6️⃣ 안전 거리 확보 후 복귀")
            movel([0, 0, 100, 0, 0, 0], vel=VELX, acc=ACCX, mod=DR_MV_MOD_REL)
            wait(0.5)
            movej(J_READY, vel=VELJ, acc=ACCJ)
            
            self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self.get_logger().info("🏁 시퀀스 완료")
            
        except Exception as e:
            self.get_logger().error(f"❌ 시퀀스 실행 중 오류: {e}")
            raise

def main(args=None):
    rclpy.init(args=args)
    
    # [중요] 1. 두산 라이브러리 기초 정보 등록
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    # [중요] 2. 노드 생성
    node = FailureRecoveryBot()
    
    # [중요] 3. 생성된 노드 객체를 라이브러리에 전달
    DR_init.__dsr__node = node 

    # [중요] 4. 노드가 등록된 '후'에 동작 함수들을 임포트하여 전역으로 설정
    global movej, movel, posx, wait, set_digital_output, DR_MV_MOD_REL
    from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_MV_MOD_REL

    try:
        node.get_logger().info("🔌 로봇 서비스 연결 확인 중...")
        node.get_logger().info("✅ 준비 완료. 토픽 대기 중...")
        # movej([0, 0, 90, 0, 100, 0], vel=VELJ, acc=ACCJ)
        # movej([0, 0, 90, 0, 90, 0], vel=VELJ, acc=ACCJ)
        # rclpy.spin은 콜백을 처리하기 위해 계속 실행됨
        
        # movej(J_READY, vel=VELJ, acc=ACCJ)
        rclpy.spin(node)
        # node.run()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 사용자에 의해 종료됨")
    except Exception as e:
        node.get_logger().error(f"❌ 예상치 못한 오류: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()