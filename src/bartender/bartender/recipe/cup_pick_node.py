#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import json
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data

# ========================================
# 로봇 설정
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Do"
ROBOT_TCP = "GripperDA_v1"

VELJ = 60
ACCJ = 60
J_READY = [0, 0, 90, 0, 90, 0]


# ========================================
# BartenderBot 클래스
# ========================================
class BartenderBot(Node):
    def __init__(self):
        super().__init__("bartender_bot", namespace=ROBOT_ID)

        self.status_pub = self.create_publisher(String, "status", 10)
        self.bridge = CvBridge()

        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        # === 경로 설정 ===
        recipe_dir = os.path.dirname(os.path.abspath(__file__))
        transform_path = os.path.join(recipe_dir, "T_gripper2camera.npy")
        recipe_path = os.path.join(recipe_dir, "recipe.json")

        # YOLO 모델
        model_path = os.path.join(recipe_dir, "best.pt")  # 학습된 best.pt
        self.yolo = YOLO(model_path)

        if os.path.exists(transform_path):
            self.gripper2cam = np.load(transform_path)
        else:
            self.get_logger().warn("⚠️ hand-eye 파일 없음 → identity 사용")
            self.gripper2cam = np.eye(4)

        # recipe.json 읽기
        with open(recipe_path, "r") as f:
            self.recipes_data = json.load(f)["recipes"]
        self.recipe_map = {r["recipe_id"]: r for r in self.recipes_data}

        # 레시피 큐 관리
        self.recipe_queue = []
        self.current_recipe = None

        # 텍스트 입력을 위한 스레드 시작
        self.input_thread = threading.Thread(target=self.input_task, daemon=True)
        self.input_thread.start()

        # 카메라 구독
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self.color_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self.info_cb, qos_profile_sensor_data
        )

        self.get_logger().info("BartenderBot 초기화 완료")

    # ===============================
    # 입력 스레드
    # ===============================
    def input_task(self):
        """콘솔에서 레시피 ID를 입력받아 큐에 추가"""
        while rclpy.ok():
            try:
                print("\n[입력] 실행할 레시피 ID를 입력하세요 (예: blue_sapphire): ", end='', flush=True)
                line = sys.stdin.readline()
                if line:
                    rid = line.strip()
                    if rid:
                        self.recipe_queue.append(rid)
            except Exception:
                time.sleep(1)

    # ===============================
    # 카메라 콜백
    # ===============================
    def color_cb(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                "fx": msg.k[0],
                "fy": msg.k[4],
                "ppx": msg.k[2],
                "ppy": msg.k[5],
            }

    # ===============================
    # 카메라 대기
    # ===============================
    def wait_camera(self):
        self.get_logger().info("카메라 데이터 대기 중...")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.color_frame is not None and self.depth_frame is not None and self.intrinsics:
                return True
        return False

    # ===============================
    # 객체 탐지
    # ===============================
    def find_object(self, target_classes=None):
        if not self.wait_camera():
            return None

        frame = self.color_frame.copy()
        results = self.yolo.predict(frame, conf=0.05, verbose=False)
        debug_img = results[0].plot()
        cv2.imwrite("debug_detection.jpg", debug_img)
        self.get_logger().info("debug_detection.jpg 저장 완료")

        if len(results[0].boxes) == 0:
            self.get_logger().warn("❌ YOLO 검출 결과 0개")
            return None

        if target_classes is None:
            target_classes = [41, 71, 39, 45]  # 컵, 병, 그릇

        candidates = [b for b in results[0].boxes if int(b.cls[0]) in target_classes]
        if not candidates:
            self.get_logger().warn("❌ 타겟 물체가 검출되지 않았습니다.")
            return None

        best = max(
            candidates,
            key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) *
                          (b.xyxy[0][3] - b.xyxy[0][1])
        )
        x1, y1, x2, y2 = best.xyxy[0].cpu().numpy()
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        roi = self.depth_frame[max(0, cy-5):min(cy+5, self.depth_frame.shape[0]),
                               max(0, cx-5):min(cx+5, self.depth_frame.shape[1])]
        valid_depths = roi[roi > 0]
        if len(valid_depths) == 0:
            self.get_logger().warn(f"❌ 중심점({cx},{cy}) 주변 깊이 정보 없음")
            return None

        depth = np.median(valid_depths)
        cam_x = (cx - self.intrinsics["ppx"]) * depth / self.intrinsics["fx"]
        cam_y = (cy - self.intrinsics["ppy"]) * depth / self.intrinsics["fy"]
        cam_z = float(depth)
        self.get_logger().info(f"픽셀=({cx},{cy}), depth={depth:.1f}, cam=({cam_x:.1f},{cam_y:.1f},{cam_z:.1f})")
        return cam_x, cam_y, cam_z

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

    def transform_to_base(self, cam_pos):
        from DSR_ROBOT2 import get_current_posx
        cx, cy, cz = cam_pos
        p_cam = np.array([cx, cy, cz, 1.0])
        p_grp = self.gripper2cam @ p_cam
        curr_pos = get_current_posx()[0]
        x, y, z, a, b, c = curr_pos
        R = Rotation.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix()
        T_base_grp = np.eye(4)
        T_base_grp[:3, :3] = R
        T_base_grp[:3, 3] = [x, y, z]
        p_base = T_base_grp @ p_grp
        return p_base[:3]

    # ===============================
    # 기본 모션
    # ===============================
    def move_to_ready(self):
        from DSR_ROBOT2 import movej, wait
        self.get_logger().info("▶ 초기 위치 이동")
        movej(J_READY, vel=VELJ, acc=ACCJ)
        wait(1.0)

    def pick_object(self, obj_class=None, hardcoded_pos=None):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL
        pos = self.find_object(target_classes=obj_class) if obj_class else None
        if pos is not None:
            bx, by, bz = self.transform_to_base(pos)
            self.get_logger().info(f"인식된 좌표: {bx:.1f}, {by:.1f}, {bz:.1f}")
        elif hardcoded_pos:
            bx, by, bz = hardcoded_pos[:3]
            self.get_logger().warn(f"인식 실패 → 하드코딩 좌표 사용: {bx},{by},{bz}")
        else:
            return None
        # 접근 & 픽
        self.release()
        rx, ry, rz = hardcoded_pos[3:] if hardcoded_pos else (0, 0, 0)
        movel(posx([bx, by, bz + 100, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        movel(posx([bx, by, bz, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.grip()
        wait(0.5)
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)
        return [bx, by, bz, rx, ry, rz]

    def pour_liquor(self, cup_pos, bottle_pos, pour_time):
        from DSR_ROBOT2 import movel, posx, wait
        cx, cy, cz = cup_pos[:3]
        brx, bry, brz = bottle_pos[3:]
        approach_z = 250
        movel(posx([cx, cy, cz + approach_z, brx, bry, brz]), vel=[100, 100], acc=[100, 100])
        wait(0.5)
        # 기울이기
        movel(posx([cx, cy, cz + approach_z, brx, bry - 45.0, brz]), vel=[40, 40], acc=[40, 40])
        self.get_logger().info(f"붓는 중: {pour_time}s")
        time.sleep(pour_time)
        # 복귀
        movel(posx([cx, cy, cz + approach_z, brx, bry, brz]), vel=[40, 40], acc=[40, 40])
        wait(0.5)

    def place_object(self, target_pos):
        """물체를 특정 위치에 내려놓는 함수"""
        from DSR_ROBOT2 import movel, posx, wait
        tx, ty, tz, rx, ry, rz = target_pos
        # 접근 (Z + 100)
        movel(posx([tx, ty, tz + 100, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        # 내려놓기 (Target Z)
        movel(posx([tx, ty, tz, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.release()
        wait(0.5)
        # 복귀 (Z + 100)
        movel(posx([tx, ty, tz + 100, rx, ry, rz]), vel=[100, 100], acc=[100, 100])

    # ===============================
    # 레시피 실행
    # ===============================
    def run_recipe(self, recipe_id):
        if recipe_id not in self.recipe_map:
            self.get_logger().warn(f"레시피 {recipe_id} 없음")
            return
        recipe = self.recipe_map[recipe_id]
        self.get_logger().info(f"=== 레시피 실행: {recipe['display_name']} ===")

        self.move_to_ready()

        # 1. 컵 픽 & 믹싱 스테이션(홈 좌표)으로 이동
        cup_hardcoded = [436, -245, 56, 19.83, 180.0, 19.28]  # 필요 시 조정
        cup_pos = self.pick_object(hardcoded_pos=cup_hardcoded)
        
        if cup_pos is None:
            self.get_logger().error("컵을 집지 못했습니다.")
            return

        # 믹싱 스테이션 좌표 (로봇 앞쪽, 홈 좌표)
        mixing_pos = [300, 0, 56, 0, 180, 0]
        self.place_object(mixing_pos)

        # 2. 각 liquors 붓기
        for liquor in recipe["liquors"]:
            # 병 픽
            # 병은 수직(180도)으로 집어야 붓기 동작(Tilt)이 자연스러움
            bottle_hardcoded = [350, 200, 130, 0, 180, 0]
            bottle_pos = self.pick_object(hardcoded_pos=bottle_hardcoded)
            
            if bottle_pos:
                # 믹싱 위치에 있는 컵에 붓기
                self.pour_liquor(mixing_pos, bottle_pos, liquor["pour_time"])
                # 병 원위치
                self.place_object(bottle_pos)

        # 3. 완료 후 홈 위치로 복귀
        self.move_to_ready()
        self.get_logger().info(f"=== 레시피 완료: {recipe['display_name']} ===")

    # ===============================
    # DB 큐 실행
    # ===============================
    def process_queue(self):
        self.get_logger().info("레시피 큐 실행 시작")
        self.get_logger().info("레시피 처리 루프 시작 (입력 대기)")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.recipe_queue and self.current_recipe is None:
                self.current_recipe = self.recipe_queue.pop(0)
                self.get_logger().info(f"실행 중: {self.current_recipe}")
                self.run_recipe(self.current_recipe)
                self.current_recipe = None
            time.sleep(0.1)

# ===============================
# main
# ===============================
def main():
    rclpy.init()
    import DR_init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = BartenderBot()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_tcp
    if get_tcp() != ROBOT_TCP:
        print(f"엔드이펙터 오류: {get_tcp()} != {ROBOT_TCP}")
        node.destroy_node()
        rclpy.shutdown()
        return
    print(f"엔드이펙터 OK: {get_tcp()}")

    try:
        node.process_queue()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
