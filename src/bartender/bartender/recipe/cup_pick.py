#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import String

# ========================================
# 로봇 설정
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v2"

VELJ = 60
ACCJ = 60
J_READY = [0, 0, 90, 0, 90, 0]


class BartenderBot(Node):
    def __init__(self):
        super().__init__("bartender_bot", namespace=ROBOT_ID)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.get_logger().info("cup_pick 노드 초기화 완료")

    def log(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def grip(self):
        from DSR_ROBOT2 import set_digital_output, wait
        # 그리퍼 닫기 (예: 1번 ON, 2번 OFF)
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait(0.8)

    def release(self):
        from DSR_ROBOT2 import set_digital_output, wait
        # 그리퍼 열기 (예: 1번 OFF, 2번 ON)
        set_digital_output(1, 0)
        set_digital_output(2, 1)
        wait(0.8)

    def initialize_robot(self):
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej,
            set_robot_mode, ROBOT_MODE_AUTONOMOUS
        )

        try:
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        except:
            pass

        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        movej(J_READY, vel=VELJ, acc=ACCJ)

        self.log("로봇 초기화 완료")

    def cup_pick_process(self):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL

        self.log("컵 픽 시작")

        # 1. 그리퍼 열기
        self.release()

        # 2. 컵 위치로 이동 (예시 좌표: 실제 환경에 맞게 수정 필요)
        # [x, y, z, rx, ry, rz]
        p_approach = [325.7, 11.10, 150.0, 19.83, 180, 19.28]
        p_pick = [325.7, 11.10, 80.0, 19.83, 180, 19.28]

        movel(posx(p_approach), vel=[100, 100], acc=[100, 100])
        movel(posx(p_pick), vel=[50, 50], acc=[50, 50])

        self.grip()

        # 3. 들어올리기 (Z축 +100mm 상대 이동)
        movel(posx([0, 0, 100, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

        self.log("컵 픽 완료")

    def run(self):
        """⚠️ spin 전에 모든 로봇 동작 실행"""
        self.initialize_robot()
        self.cup_pick_process()
        self.log("cup_pick 프로세스 종료")


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = BartenderBot()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_tcp
    print(f"현재 TCP: {get_tcp()}")

    try:
        time.sleep(1.0)
        node.run()

        # 상태 publish용으로만 spin (로봇 호출 없음)
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("사용자 중단")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
