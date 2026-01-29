#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import rclpy
from rclpy.node import Node
import DR_init
from std_msgs.msg import String

# 비전 관련 import
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
ROBOT_TCP = "GripperDA_v2"

VELJ = 60
ACCJ = 60
J_READY = [0, 0, 90, 0, 90, 0]


class BartenderBot(Node):
    def __init__(self):
        super().__init__("bartender_bot", namespace=ROBOT_ID)

        # --- 기존 설정 ---
        self.status_pub = self.create_publisher(String, "status", 10)
        self.get_logger().info("cup_pick 노드 초기화 완료")

        # --- 비전 모듈 설정 ---
        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        # ROS 2 패키지 공유 디렉토리에서 파일 경로 찾기
        package_share_directory = get_package_share_directory('bartender')
        recipe_dir = os.path.join(package_share_directory, 'recipe')
        transform_path = os.path.join(recipe_dir, "T_gripper2camera.npy")
        model_path = "/home/dabom/dynamic_busan/src/bartender/bartender/recipe/yolov8n.pt"

        if os.path.exists(transform_path):
            self.gripper2cam_transform = np.load(transform_path)
            self.get_logger().info(f"캘리브레이션 데이터 로드 완료: {transform_path}")
        else:
            self.get_logger().warn(f"⚠️ 캘리브레이션 파일이 없습니다: {transform_path}")
            self.get_logger().warn("기본 단위 행렬을 사용합니다. 좌표 변환이 정확하지 않습니다.")
            self.gripper2cam_transform = np.eye(4)

        self.yolo_model = YOLO(model_path)

        self.cup_class_id = 47  # COCO dataset에서 'cup'의 ID

        # 카메라 토픽 구독
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, qos_profile_sensor_data)

        self.get_logger().info("cup_pick 노드 초기화 완료 (비전 모듈 포함)")

    # --- 로봇 및 로깅 관련 함수 ---
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

    # --- 카메라 콜백 함수 ---
    def color_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}
            self.get_logger().info(f"카메라 내부 파라미터 수신: {self.intrinsics}")

    def wait_for_camera_data(self):
        self.log("카메라 데이터 수신 대기 중...")
        wait_count = 0
        while rclpy.ok() and (self.color_frame is None or self.depth_frame is None or self.intrinsics is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            wait_count += 1

            # 5초마다 토픽 상태 진단
            if wait_count % 50 == 0:
                topic_names = [t[0] for t in self.get_topic_names_and_types()]
                camera_topics = [t for t in topic_names if 'image_raw' in t]
                
                if not camera_topics:
                    self.get_logger().warn("⚠️ 카메라 토픽('image_raw')이 발견되지 않습니다. 카메라 노드를 실행했나요?")
                else:
                    self.get_logger().info(f"ℹ️ 발견된 카메라 토픽 목록: {camera_topics}")
                    target_topic = '/camera/camera/color/image_raw'
                    if target_topic not in camera_topics:
                        self.get_logger().warn(f"⚠️ 코드의 토픽('{target_topic}')과 일치하는 토픽이 없습니다. 위 목록을 참고하여 코드를 수정하세요.")

            if wait_count % 30 == 0:  # 약 3초마다 상태 출력
                self.get_logger().info(f"데이터 수신 대기 중... [Color: {'OK' if self.color_frame is not None else 'NO'}, Depth: {'OK' if self.depth_frame is not None else 'NO'}, Info: {'OK' if self.intrinsics is not None else 'NO'}]")
        if not rclpy.ok():
            return False

        # 데이터 수신 완료 후 안전하게 구독 해제 (콜백 내부가 아닌 여기서 수행)
        if self.cam_info_sub is not None:
            self.destroy_subscription(self.cam_info_sub)
            self.cam_info_sub = None

        self.log("카메라 데이터 수신 완료.")
        return True

    # --- 비전 처리 및 좌표 변환 함수 ---
    def find_cup(self):
        """YOLO로 컵을 탐지하고 로봇 베이스 좌표계 기준 3D 위치를 반환합니다."""
        if not self.wait_for_camera_data():
            return None

        frame = self.color_frame.copy()
        # 감지 민감도를 높이기 위해 conf를 0.5 -> 0.25로 낮춤
        results = self.yolo_model.predict(frame, conf=0.15, classes=[self.cup_class_id], verbose=False)

        # 디버깅용 이미지 저장 (현재 터미널 실행 위치에 저장됨)
        debug_image_path = "debug_detection.jpg"
        cv2.imwrite(debug_image_path, results[0].plot())
        self.log(f"디버그 이미지 저장: {os.path.abspath(debug_image_path)}")

        if len(results[0].boxes) == 0:
            self.log("컵을 찾지 못했습니다. 저장된 debug_detection.jpg 이미지를 확인해보세요.")
            return None

        # 가장 큰 컵을 대상으로 선택
        best_box = max(results[0].boxes, key=lambda box: (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1]))
        xyxy = best_box.xyxy[0].cpu().numpy()

        # 바운딩 박스의 중심점 계산
        cx = int((xyxy[0] + xyxy[2]) / 2)
        cy = int((xyxy[1] + xyxy[3]) / 2)

        # 중심점의 깊이 값 확인
        depth = self.depth_frame[cy, cx]
        if depth == 0:
            self.log(f"컵의 깊이 정보를 얻을 수 없습니다 (좌표: {cx},{cy}).")
            return None

        self.log(f"컵 감지 성공. 픽셀 좌표: ({cx}, {cy}), 깊이: {depth}mm")

        # 3D 카메라 좌표로 변환 (단위: mm)
        cam_x = (cx - self.intrinsics["ppx"]) * depth / self.intrinsics["fx"]
        cam_y = (cy - self.intrinsics["ppy"]) * depth / self.intrinsics["fy"]
        cam_z = float(depth)
        camera_coords = (cam_x, cam_y, cam_z)
        self.log(f"카메라 좌표계 기준 위치: {camera_coords}")

        # 로봇 베이스 좌표로 변환
        base_coords = self.transform_to_base(camera_coords)
        self.log(f"로봇 베이스 좌표계 기준 위치: {base_coords}")

        return base_coords

    def transform_to_base(self, camera_coords):
        """3D 카메라 좌표를 로봇 베이스 좌표계로 변환합니다."""
        from DSR_ROBOT2 import get_current_posx

        # 카메라 좌표를 동차좌표로 변환
        cam_point_h = np.append(np.array(camera_coords), 1)

        # 현재 로봇 자세 (베이스 -> 그리퍼)
        current_posx = get_current_posx()[0]
        base2gripper_transform = self.get_robot_pose_matrix(*current_posx)

        # 베이스 -> 카메라 변환 행렬 계산
        base2cam_transform = base2gripper_transform @ self.gripper2cam_transform

        # 카메라 좌표를 베이스 좌표로 변환
        base_point_h = base2cam_transform @ cam_point_h

        return base_point_h[:3]

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        """로봇 좌표(x,y,z,rx,ry,rz)로부터 4x4 변환 행렬을 생성합니다."""
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    # --- 메인 동작 함수 ---
    def cup_pick_process(self):
        from DSR_ROBOT2 import movej, movel, posx, wait, DR_MV_MOD_REL

        self.log("컵 픽 시작 (비전 기반)")

        # 1. 스캔 위치로 이동
        self.log("스캔 위치로 이동합니다.")
        movej(J_READY, vel=VELJ, acc=ACCJ)
        wait(1.0)  # 안정화 대기

        # 2. 컵 찾기 (위치 변환 포함)
        cup_position = self.find_cup()
        if cup_position is None:
            self.log("컵 픽 프로세스 중단: 컵을 찾을 수 없습니다.")
            return

        x, y, z = cup_position
        self.log(f"감지된 컵 좌표: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # 3. 그리퍼 열기
        self.release()

        # 4. 컵 위치로 이동 좌표 계산
        pick_orientation = [19.83, 180, 19.28]  # 아래를 향하는 방향 (예시)
        rx, ry, rz = pick_orientation

        approach_z_offset = 100.0  # 컵 위 100mm에서 접근
        pick_z_offset = -20.0  # 감지된 컵 상단보다 20mm 아래를 잡음 (튜닝 필요)

        p_approach = [x, y, z + approach_z_offset, rx, ry, rz]
        p_pick = [x, y, z + pick_z_offset, rx, ry, rz]

        self.log(f"접근 위치로 이동: {p_approach}")
        movel(posx(p_approach), vel=[100, 100], acc=[100, 100])
        self.log(f"픽업 위치로 이동: {p_pick}")
        movel(posx(p_pick), vel=[50, 50], acc=[50, 50])

        # 5. 컵 잡기
        self.grip()
        wait(0.5)

        # 6. 들어올리기
        self.log("컵 들어올리기")
        movel(posx([0, 0, 150, 0, 0, 0]), vel=[100, 100], acc=[100, 100], mod=DR_MV_MOD_REL)

        # 7. 준비 자세로 복귀 후 컵 놓기
        self.log("준비 자세로 복귀합니다.")
        movej(J_READY, vel=VELJ, acc=ACCJ)
        self.release()
        self.log("cup_pick 프로세스 완료")

    def run(self):
        """⚠️ spin 전에 모든 로봇 동작 실행"""
        self.initialize_robot()
        self.cup_pick_process()
        self.log("모든 동작 완료. 노드는 상태 publish를 위해 계속 실행됩니다.")


def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = BartenderBot()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_tcp

    try:
        # 로봇 초기화 및 메인 프로세스 실행
        node.run()

        # 상태 publish용으로만 spin (로봇 호출 없음)
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 중단 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
