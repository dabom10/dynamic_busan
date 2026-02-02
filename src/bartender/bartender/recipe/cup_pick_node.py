#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import String

import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data


# ========================================
# 로봇 설정
# ========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELJ = 60
ACCJ = 60
J_READY = [0, 0, 90, 0, 90, 0]


class BartenderBot(Node):
    def __init__(self):
        super().__init__("bartender_bot", namespace=ROBOT_ID)

        self.status_pub = self.create_publisher(String, "status", 10)
        self.bridge = CvBridge()

        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        # === 경로 설정 ===
        pkg_share = get_package_share_directory("bartender")
        recipe_dir = os.path.join(pkg_share, "recipe")
        transform_path = os.path.join(recipe_dir, "T_gripper2camera.npy")

        # YOLO 모델 (절대경로, 디버깅용)
        model_path = "/home/dabom/dynamic_busan/src/bartender/bartender/recipe/yolov8n.pt"
        self.yolo = YOLO(model_path)

        if os.path.exists(transform_path):
            self.gripper2cam = np.load(transform_path)
        else:
            self.get_logger().warn("⚠️ hand-eye 파일 없음 → identity 사용")
            self.gripper2cam = np.eye(4)

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
    # 콜백
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
    # 컵 탐지 (디버깅 핵심)
    # ===============================
    def find_object(self):
        if not self.wait_camera():
            return None

        frame = self.color_frame.copy()

        results = self.yolo.predict(
            frame,
            conf=0.05,      # 🔥 극단적으로 낮춤 (진단용)
            verbose=False
        )

        debug_img = results[0].plot()
        cv2.imwrite("debug_detection.jpg", debug_img)
        self.get_logger().info("debug_detection.jpg 저장 완료")

        if len(results[0].boxes) == 0:
            self.get_logger().warn("❌ YOLO 검출 결과 0개")
            return None

        # 모든 검출 로그 출력
        for box in results[0].boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            self.get_logger().info(f"Detected class={cls}, conf={conf:.2f}")

        # 1. 타겟 클래스만 필터링 (41:cup, 39:bottle, 45:bowl)
        target_classes = [41, 71, 39, 45]
        candidates = [b for b in results[0].boxes if int(b.cls[0]) in target_classes]

        if not candidates:
            self.get_logger().warn("❌ 타겟 물체(컵, 병, 그릇)가 검출되지 않았습니다.")
            return None

        # 가장 큰 박스를 컵 후보로 선택
        best = max(
            candidates,
            key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) *
                          (b.xyxy[0][3] - b.xyxy[0][1])
        )

        x1, y1, x2, y2 = best.xyxy[0].cpu().numpy()
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        # 2. 깊이값 보정 (중심점 주변 10x10 영역의 중앙값 사용)
        # depth=0인 픽셀(결측치)을 제외하고 계산하여 안정성 확보
        roi = self.depth_frame[max(0, cy-5):min(cy+5, self.depth_frame.shape[0]), max(0, cx-5):min(cx+5, self.depth_frame.shape[1])]
        valid_depths = roi[roi > 0]

        if len(valid_depths) == 0:
            self.get_logger().warn(f"❌ 중심점({cx},{cy}) 주변 깊이 정보 없음 (depth=0)")
            return None

        depth = np.median(valid_depths)
        cam_x = (cx - self.intrinsics["ppx"]) * depth / self.intrinsics["fx"]
        cam_y = (cy - self.intrinsics["ppy"]) * depth / self.intrinsics["fy"]
        cam_z = float(depth)

        self.get_logger().info(
            f"픽셀=({cx},{cy}), depth={depth}mm, cam=({cam_x:.1f},{cam_y:.1f},{cam_z:.1f})"
        )

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
        # 1. 카메라 좌표계 점 (Homogeneous)
        p_cam = np.array([cx, cy, cz, 1.0])
        
        # 2. Gripper 좌표계로 변환 (Hand-Eye Calibration)
        p_grp = self.gripper2cam @ p_cam
        
        # 3. Base 좌표계로 변환 (Robot Kinematics)
        # 현재 로봇 위치 가져오기 (x, y, z, a, b, c) - Euler ZYZ
        curr_pos = get_current_posx()[0]
        x, y, z, a, b, c = curr_pos
        
        # 회전 행렬 생성 (Doosan은 ZYZ Euler angle 사용)
        R = Rotation.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix()
        T_base_grp = np.eye(4)
        T_base_grp[:3, :3] = R
        T_base_grp[:3, 3] = [x, y, z]
        
        p_base = T_base_grp @ p_grp
        return p_base[:3]

    # ===============================
    # 실행
    # ===============================

    def move_to_ready(self):
        from DSR_ROBOT2 import movej, wait
        self.get_logger().info("▶ [1/7] 초기 위치 이동")
        movej(J_READY, vel=VELJ, acc=ACCJ)
        wait(1.0)

    def pick_cup(self):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL
        self.get_logger().info("▶ [2/7] 컵 픽 & 배치 (Pouring Position)")

        # 1. 컵 인식
        pos = self.find_object()
        
        # 좌표 설정 (인식 실패 시 하드코딩)
        if pos is not None:
            bx, by, bz = self.transform_to_base(pos)
            self.get_logger().info(f"   - 인식된 좌표: {bx:.1f}, {by:.1f}, {bz:.1f}")
        else:
            self.get_logger().warn("   - 인식 실패: 하드코딩 좌표 사용")
            bx, by, bz = 436.0, -245.0, 56.0 # 예시 좌표

        rx, ry, rz = 19.83, 180.0, 19.28

        # 접근 및 픽
        self.release()
        movel(posx([bx, by, bz + 100, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        movel(posx([bx, by, bz - 20, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.grip()
        wait(0.5)

        # 리프트
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

        # 붓기 위치(중앙)로 이동 및 배치
        # 붓기 편한 위치로 설정
        self.pour_pos = [350, 0, 150.0, rx, ry, rz] 
        px, py, pz, prx, pry, prz = self.pour_pos

        movel(posx([px, py, pz + 150, prx, pry, prz]), vel=[100, 100], acc=[100, 100])
        movel(posx(self.pour_pos), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.release()
        wait(0.5)
        
        # 안전 높이로 후퇴
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

    def pick_bottle(self):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL
        self.get_logger().info("▶ [3/7] 병 픽")

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
        self.get_logger().info("▶ [4/7] 붓기 동작")

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
        self.get_logger().info("▶ [5/7] 병 원위치 복귀")

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

    def place_cup_home(self):
        from DSR_ROBOT2 import movel, posx, wait, DR_MV_MOD_REL
        self.get_logger().info("▶ [6/7] 컵 복귀")

        # 붓기 위치의 컵 잡기
        cx, cy, cz, rx, ry, rz = self.pour_pos

        self.release()
        movel(posx([cx, cy, cz + 150, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        movel(posx([cx, cy, cz, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.grip()
        wait(0.5)
        
        # 리프트
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

        # 원래 위치(또는 지정된 홈)로 이동
        # 여기서는 간단히 오른쪽 구석으로 이동
        tx, ty, tz = 350, -200, 56.0
        
        movel(posx([tx, ty, tz + 150, rx, ry, rz]), vel=[100, 100], acc=[100, 100])
        movel(posx([tx, ty, tz, rx, ry, rz]), vel=[50, 50], acc=[50, 50])
        wait(0.5)
        self.release()
        wait(0.5)
        
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

    def run(self):
        self.get_logger().info("=== 바텐더 모션 검증 시작 ===")
        
        # 순서대로 실행하는 거 if문으로 만들기(run 안에서 바꾸면 되겠다) --> 액션하려고 
        self.move_to_ready()
        self.pick_cup()
        self.pick_bottle()
        self.pour_motion()
        self.place_bottle_back()
        self.place_cup_home()
        self.move_to_ready()
        
        self.get_logger().info("=== 바텐더 모션 검증 완료 ===")


def main():
    rclpy.init()
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = BartenderBot()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_tcp
    # 조건 불일치 → offset 값 올리기
    if get_tcp() != ROBOT_TCP:
        print(f"엔드이펙터 - Gripper 오류: {get_tcp()} != {ROBOT_TCP}")
        node.destroy_node()
        rclpy.shutdown()
        return
    print(f"엔드이펙터 - Gripper : {get_tcp()}")

    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()