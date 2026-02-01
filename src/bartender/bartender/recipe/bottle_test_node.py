#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import DR_init

# ========================================
# 로봇 설정
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "GripperDA_v1"

class BottleTestNode(Node):
    def __init__(self):
        super().__init__("bottle_test_node", namespace=ROBOT_ID)
        self.get_logger().info("Bottle Test Node Initialized")
        
        # pour_motion에서 참조하는 컵 위치 (pick_cup을 건너뛰므로 수동 설정)
        # pick_cup에서 설정된 값: [350, 0, 150.0, 19.83, 180.0, 19.28]
        self.pour_pos = [350, 0, 150.0, 19.83, 180.0, 19.28]
        self.bottle_origin = None

    # ===============================
    # 로봇 유틸리티
    # ===============================
    def grip(self):
        from DSR_ROBOT2 import set_digital_output
        self.get_logger().info("GRIP ON")
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        time.sleep(0.3)

    def release(self):
        from DSR_ROBOT2 import set_digital_output
        self.get_logger().info("GRIP OFF")
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        time.sleep(0.3)

    # ===============================
    # 테스트할 함수들
    # ===============================
    def pick_bottle(self):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL
        self.get_logger().info("▶ [1/3] 병 픽")

        # 병 위치 (하드코딩) - 컵과 다른 위치
        bx, by, bz = 350, 200, 130.0
        # 수평 집기 (Ry=-90도 가정)
        rx, ry, rz = 0.0, -90.0, 0.0
        self.bottle_origin = [bx, by, bz, rx, ry, rz]

        self.release()
        # 접근
        movel(posx([bx, by, bz + 150, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        # 픽
        movel(posx([bx, by, bz, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.grip()
        wait(0.5)
        # 리프트
        movel(posx([0, 0, 200, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

    def pour_motion(self):
        from DSR_ROBOT2 import movel, posx, wait
        self.get_logger().info("▶ [2/3] 붓기 동작")

        # 컵 위치 위로 이동
        # 컵의 위치(xyz)만 가져오고, 회전(rpy)은 병을 잡은 상태(bottle_origin)를 기준
        cx, cy, cz = self.pour_pos[:3]
        brx, bry, brz = self.bottle_origin[3:]
        
        approach_z = 250 # 컵보다 충분히 높게

        # 1. 컵 상공 이동 (병 자세 유지)
        movel(posx([cx, cy, cz + approach_z, brx, bry, brz]), vel=[100, 100], acc=[100, 100])
        wait(0.5)

        # 2. 기울이기 (Pitch 회전)
        # 수평(-90) -> 붓기 각도(예: -45도 -> -135도)
        self.get_logger().info("   - 기울이기...")
        movel(posx([cx, cy, cz + approach_z, brx, bry - 45.0, brz]), vel=[40, 40], acc=[40, 40])
        
        # 3. 붓기 시간
        self.get_logger().info("   - 붓는 중 (3초)")
        time.sleep(3.0)

        # 4. 복귀
        self.get_logger().info("   - 복귀")
        movel(posx([cx, cy, cz + approach_z, brx, bry, brz]), vel=[40, 40], acc=[40, 40])
        wait(0.5)

    def place_bottle_back(self):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL
        self.get_logger().info("▶ [3/3] 병 원위치 복귀")

        bx, by, bz, rx, ry, rz = self.bottle_origin

        # 접근
        movel(posx([bx, by, bz + 150, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        # 놓기
        movel(posx([bx, by, bz, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.release()
        wait(0.5)
        # 퇴장
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

    def run(self):
        self.get_logger().info("=== 병 붓기 모션 테스트 시작 ===")
        
        from DSR_ROBOT2 import movej, wait
        # 초기 위치 이동
        J_READY = [0, 0, 90, 0, 90, 0]
        movej(J_READY, vel=60, acc=60)
        wait(1.0)

        # 테스트 시퀀스 실행
        self.pick_bottle()
        self.pour_motion()
        self.place_bottle_back()
        
        # 초기 위치 복귀
        movej(J_READY, vel=60, acc=60)
        wait(1.0)
        
        self.get_logger().info("=== 병 붓기 모션 테스트 완료 ===")


def main():
    rclpy.init()
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = BottleTestNode()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_tcp, set_tcp
    
    # TCP 설정 확인 및 강제 설정
    if get_tcp() != ROBOT_TCP:
        print(f"엔드이펙터 불일치: {get_tcp()} != {ROBOT_TCP}")
        print(f"TCP 설정을 {ROBOT_TCP}로 강제 변경합니다.")
        set_tcp(ROBOT_TCP)
    else:
        print(f"엔드이펙터 확인됨: {get_tcp()}")

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()