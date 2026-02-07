#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bartender_interfaces.action import Motion
from bartender_interfaces.srv import DrinkDelivery  # 서비스 통신용
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import DR_init
from bartender.onrobot import RG
from bartender.db.db_client import DBClient
import os
import sys
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import threading

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "GripperDA_v1"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
BASE_HOME_POS = [389.39, 21.52, 55.59, 10.74, -179.71, 10.58]
#"green_cup": 145.0,
#"black_cup": 80.0,
#"yellow_cup": 50.0

# 그리퍼 초기화
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# DSR 메시지 import (같은 워크스페이스에서 빌드 필요)
try:
    from dsr_msgs2.srv import MoveLine, MoveJoint
    from dsr_msgs2.srv import SetCtrlBoxDigitalOutput, SetCurrentTool
    try:
        from dsr_msgs2.srv import GetCurrentPose as GetCurrentPos
    except ImportError:
        from dsr_msgs2.srv import GetCurrentPose as GetCurrentPos
except ImportError as e:
    print(f"ERROR: dsr_msgs2 import 실패: {e}")
    print("해결방법: colcon build로 전체 워크스페이스를 빌드한 후 source install/setup.bash 실행")
    sys.exit(1)


class ShakeController(Node):
    def __init__(self):
        super().__init__("shake_node", namespace=ROBOT_ID)
        self.get_logger().info("=== Shake Node (Vision + Motion + Delivery) ===")

        # 파일 경로 설정
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'shake.pt')
        # 캘리브레이션 파일은 recipe 폴더에서 참조
        recipe_dir = os.path.join(os.path.dirname(current_dir), 'recipe')
        calib_path = os.path.join(recipe_dir, 'T_gripper2camera.npy')

        # 캘리브레이션 매트릭스 로드
        if os.path.exists(calib_path):
            T_gripper2camera = np.load(calib_path)
            # 카메라 좌표 → 그리퍼 좌표 변환을 위해 역행렬 사용
            self.calib_matrix = np.linalg.inv(T_gripper2camera)
        else:
            self.calib_matrix = np.eye(4)
            self.get_logger().warn("캘리브레이션 파일 없음. 단위 행렬 사용.")

        # YOLO 모델 로드
        self.model = YOLO(model_path)
        # RealSense (필요할 때만 열고 닫음)
        self.pipeline = None
        self.align = None
        self.depth_scale = None

        # Callback Group 생성 (Action과 DB 응답을 동시 처리)
        self._callback_group = ReentrantCallbackGroup()
        self.get_logger().info(f"🔧 ReentrantCallbackGroup 생성됨: {self._callback_group}")

        # DB 클라이언트 초기화 (callback_group 전달)
        self.db_client = DBClient(self, callback_group=self._callback_group)
        self.db_query_result = []
        self.db_query_event = threading.Event()
        self.get_logger().info("✅ DBClient 초기화 완료 (callback_group 전달)")

        # ROS 퍼블리셔/클라이언트
        self.pub_img = self.create_publisher(Image, '/shake/yolo_image', 10)
        self.br = CvBridge()

        # Service 클라이언트에 callback_group 설정
        self.move_line_client = self.create_client(
            MoveLine, '/dsr01/motion/move_line', callback_group=self._callback_group)
        self.move_joint_client = self.create_client(
            MoveJoint, '/dsr01/motion/move_joint', callback_group=self._callback_group)
        self.get_pos_client = self.create_client(
            GetCurrentPos, '/dsr01/system/get_current_pose', callback_group=self._callback_group)
        self.set_tool_client = self.create_client(
            SetCurrentTool, '/dsr01/system/set_current_tool', callback_group=self._callback_group)

        self.delivery_client = self.create_client(DrinkDelivery, 'get_pose')

        # Subscriber (current_menu 구독)
        self.sub_current_menu = self.create_subscription(
            String, '/current_menu', self.on_current_menu, 10)

        # Action Server 생성 (같은 callback_group 사용)
        self._action_server = ActionServer(
            self,
            Motion,
            'shake/motion',
            self.execute_callback,
            callback_group=self._callback_group
        )

        # 상태 변수
        self.is_running = False
        self.target_object = "muddler"
        self.status_msg = "Ready"
        self.shaker_origin_pos = None
        self.cup_type = "green_cup"  # 기본값

        # 컵 높이 정의
        self.CUP_HEIGHTS = {
            "green_cup": 140.0,
            "black_cup": 80.0,
            "yellow_cup": 50.0
        }

        # 위치 파라미터 (TODO: 실제 환경에 맞게 조정)
        self.JOINT_HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.SEARCH_POS = [-18.0, 44.0, 78.0, 74.0, 99.0, -36.0]  # shaker 탐색 위치
        self.SHAKE_POS_1 = [350.0, 0.0, 300.0, 45.0, 135.0, 45.0]  # 쉐이킹 자세 1
        self.SHAKE_POS_2 = [350.0, 0.0, 300.0, -45.0, -135.0, -45.0]  # 쉐이킹 자세 2

        # TCP 설정
        self.set_robot_tcp()

        self.get_logger().info("Shake Action Server ready (shake/motion)")

    def start_camera(self):
        """RealSense 카메라 시작 (재시도 로직 포함)"""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"📷 RealSense 시작 시도 ({attempt + 1}/{max_retries})")

                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

                profile = self.pipeline.start(config)
                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()
                self.align = rs.align(rs.stream.color)

                # Auto Exposure 활성화 (어두운 화면 문제 해결)
                try:
                    color_sensor = profile.get_device().first_color_sensor()
                    if color_sensor.supports(rs.option.enable_auto_exposure):
                        color_sensor.set_option(rs.option.enable_auto_exposure, 1.0)
                        self.get_logger().info("✅ Auto Exposure 활성화")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Auto Exposure 설정 실패: {e}")

                self.get_logger().info("✅ RealSense 시작 성공")

                # 첫 프레임 대기 (카메라 워밍업)
                time.sleep(0.5)
                return True

            except Exception as e:
                self.get_logger().error(f"❌ RealSense 시작 실패 (시도 {attempt + 1}): {e}")
                self.stop_camera()

        return False

    def stop_camera(self):
        """RealSense 카메라 안전하게 종료"""
        if self.pipeline:
            try:
                self.pipeline.stop()
                self.get_logger().info("✅ RealSense 종료")
            except Exception as e:
                self.get_logger().warn(f"⚠️ RealSense 종료 중 에러 (무시됨): {e}")
            finally:
                self.pipeline = None
                self.align = None
                self.depth_scale = None

    def set_robot_tcp(self):
        if self.set_tool_client.wait_for_service(timeout_sec=1.0):
            req = SetCurrentTool.Request()
            req.name = ROBOT_TCP
            self.set_tool_client.call_async(req)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Action 실행 콜백"""
        motion_name = goal_handle.request.motion_name
        self.get_logger().info(f"🎯 Shake Goal 수신: {motion_name}")

        if self.is_running:
            self.get_logger().warn("이미 실행 중입니다.")
            goal_handle.abort()
            return Motion.Result(success=False, message="Already running")

        self.is_running = True
        feedback_msg = Motion.Feedback()
        start_time = time.time()

        try:
            self.stop_camera()
            # 1. 홈 위치로 이동
            self.publish_feedback(goal_handle, feedback_msg, 10, "홈 위치 이동")
            if not self.move_to_joint(self.JOINT_HOME):
                raise Exception("홈 위치 이동 실패")

            # 2. 탐색 위치로 이동
            self.publish_feedback(goal_handle, feedback_msg, 20, "탐색 위치 이동")
            if not self.move_to_joint(self.SEARCH_POS):
                raise Exception("탐색 위치 이동 실패")

            # 3. 객체 탐색 및 그립
            self.publish_feedback(goal_handle, feedback_msg, 30, "Shaker 탐색 중...")

            # 카메라 시작 (실패 시 중단)
            if not self.start_camera():
                raise Exception("RealSense 카메라 시작 실패")
            
            time.sleep(0.5)

            detection_result = self.detect_and_approach()
            
            if not detection_result:
                raise Exception("Shaker 탐색 실패")

            # 4. 그립
            self.publish_feedback(goal_handle, feedback_msg, 50, "Shaker 그립")
            gripper.close_gripper()
            time.sleep(1.0)

            # 5. 들어올리기
            self.publish_feedback(goal_handle, feedback_msg, 55, "들어올리기")
            if not self.lift_object(220.0):
                raise Exception("들어올리기 실패")

            # 6. 쉐이킹 모션
           # self.publish_feedback(goal_handle, feedback_msg, 60, "쉐이킹 시작")
            shake_count = 1  # 쉐이킹 횟수
            for i in range(shake_count):
                progress = 60 + int((i + 1) / shake_count * 25)
                self.publish_feedback(goal_handle, feedback_msg, progress, f"쉐이킹 {i+1}/{shake_count}")
                if not self.shake_motion():
                    raise Exception(f"쉐이킹 {i+1} 실패")

            # 7. 원위치 복귀
            self.publish_feedback(goal_handle, feedback_msg, 88, "원위치 복귀")
            if not self.return_shaker():
                raise Exception("원위치 복귀 실패")

            # 8. 그리퍼 해제 및 후퇴
            self.publish_feedback(goal_handle, feedback_msg, 88, "그리퍼 해제")
            gripper.open_gripper()
            time.sleep(0.5)
            self.retract(100.0)

            # 9. 컵 집기
            self.publish_feedback(goal_handle, feedback_msg, 90, "컵 집기")
            if not self.pick_cup(self.cup_type):
                raise Exception("컵 집기 실패")

            # 10. 고객 위치로 전달
            self.publish_feedback(goal_handle, feedback_msg, 95, "고객 위치로 전달 중...")
            delivery_success = self.send_delivery_request()

            if not delivery_success:
                self.get_logger().warn("⚠️ 음료 전달 실패. 홈으로 복귀합니다.")

            # 11. 홈으로 복귀
            self.publish_feedback(goal_handle, feedback_msg, 100, "완료")
            self.move_to_joint(self.JOINT_HOME)

            elapsed_ms = int((time.time() - start_time) * 1000)
            goal_handle.succeed()

            result = Motion.Result()
            result.success = True
            result.message = f"Shake motion '{motion_name}' 완료"
            result.total_time_ms = elapsed_ms
            self.get_logger().info(f"✅ 쉐이킹 및 전달 완료: {elapsed_ms}ms")

        except Exception as e:
            self.get_logger().error(f"❌ 쉐이킹 실패: {e}")
            goal_handle.abort()
            result = Motion.Result()
            result.success = False
            result.message = str(e)
            result.total_time_ms = int((time.time() - start_time) * 1000)

        finally:
            self.stop_camera()
            self.is_running = False

        return result

    def send_delivery_request(self):
        from DSR_ROBOT2 import movel, posx, movej

        # 서비스 연결 확인
        if not self.delivery_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("⚠️ DrinkDelivery 서비스 연결 실패. 전달 건너뜀.")
            return False

        # 요청 생성
        req = DrinkDelivery.Request()
        req.finish = True  # 제작 완료 신호

        self.get_logger().info("📤 DrinkDelivery 요청 전송...")

        # 동기 호출 (응답 대기)
        future = self.delivery_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.result():
            self.get_logger().error("❌ DrinkDelivery 응답 없음")
            return False

        response = future.result()
        pos = list(response.goal_position)

        self.get_logger().info(f"📍 받은 고객 위치: {pos}")

        # 위치가 없으면 종료
        if len(pos) <= 0:
            self.get_logger().warn("⚠️ 반환된 좌표가 없습니다")
            return False

        try:
            # 홈 위치로 이동 (안전)
            self.get_logger().info("🏠 홈 위치로 이동")
            movej([0, 0, 90, 0, 90, 0], vel=60, acc=60)

            # 고객 위치로 이동
            self.get_logger().info(f"🚀 고객 위치로 이동: {pos}")
            movel(posx(pos), vel=60, acc=60)
            gripper.open_gripper()
            time.sleep(5.0)

            self.get_logger().info("✅ 음료 전달 완료")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ 음료 전달 중 에러: {e}")
            return False

    def publish_feedback(self, goal_handle, feedback_msg, progress, step):
        feedback_msg.progress = progress
        feedback_msg.current_step = step
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f"📢 [{progress}%] {step}")

    def detect_and_approach(self, timeout=10.0):
        """객체 탐색 및 접근"""
        self.get_logger().info(f"🔎 객체 탐색 시작 (대상: {self.target_object}, 제한시간: {timeout}초)")

        # 카메라 파이프라인 확인
        if self.pipeline is None:
            self.get_logger().error("❌ 카메라 파이프라인이 초기화되지 않았습니다")
            return False

        start_time = time.time()

        frame_count = 0
        consecutive_failures = 0  # 연속 실패 카운트
        max_consecutive_failures = 5  # 최대 연속 실패 허용

        while time.time() - start_time < timeout:
            try:
                # 프레임 획득 (타임아웃 줄임: 2000ms → 1000ms)
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                if not frames:
                    consecutive_failures += 1
                    self.get_logger().warn(f"⚠️ 프레임 획득 실패 ({consecutive_failures}/{max_consecutive_failures})")

                    if consecutive_failures >= max_consecutive_failures:
                        self.get_logger().error("❌ 카메라 응답 없음. 연속 실패 한계 도달")
                        return False
                    continue

                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    consecutive_failures += 1
                    self.get_logger().warn(f"⚠️ 컬러 또는 뎁스 프레임 없음 ({consecutive_failures}/{max_consecutive_failures})")

                    if consecutive_failures >= max_consecutive_failures:
                        self.get_logger().error("❌ 유효한 프레임을 받을 수 없음")
                        return False
                    continue

                # 프레임 정상 획득 - 실패 카운터 리셋
                consecutive_failures = 0

            except Exception as e:
                consecutive_failures += 1
                self.get_logger().error(f"❌ 카메라 에러: {e} ({consecutive_failures}/{max_consecutive_failures})")

                if consecutive_failures >= max_consecutive_failures:
                    self.get_logger().error("❌ 카메라 에러 한계 도달. 탐색 중단")
                    return False
                time.sleep(0.1)
                continue

            frame_count += 1
            img = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # YOLO 추론 (매우 낮은 신뢰도도 탐지하도록 conf 임계값 최소화)
            results = self.model(img, verbose=False, conf=0.1)  # 신뢰도 10% 이상만 탐지
            annotated_frame = results[0].plot()

            # 상태 표시
            cv2.rectangle(annotated_frame, (0, 0), (640, 40), (0, 0, 0), -1)
            cv2.putText(annotated_frame, f"Searching: {self.target_object}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 이미지 퍼블리시
            msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.pub_img.publish(msg)

            # RGB 이미지에 YOLO 탐지 결과 표시 (cup_pick_node 방식)
            cv2.imshow("Shake Vision", annotated_frame)
            cv2.waitKey(1)

            # 객체 탐지
            boxes = results[0].boxes

            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])

                # target_object와 비교 (대소문자 무시)
                if cls_name.lower() == self.target_object.lower():
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # 바운딩 박스의 중심점 사용 (단순하고 안정적)
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # 화면에 중심점 표시
                    cv2.circle(annotated_frame, (cx, cy), 5, (255, 0, 255), -1)
                    cv2.putText(annotated_frame, f"Conf: {conf:.2f}",
                               (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Depth 측정
                    dist = self.get_depth_at_point(depth_image, cx, cy)

                    if dist == 0.0:
                        self.get_logger().warn(f"⚠️ Depth 측정 실패")
                        continue

                    # 3D 좌표 계산 (카메라 좌표계)
                    cam_point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)
                    c_x = cam_point[0] * 1000.0
                    c_y = cam_point[1] * 1000.0
                    c_z = cam_point[2] * 1000.0

                    # 그리퍼 좌표로 변환
                    gripper_pos = np.dot(self.calib_matrix, np.array([c_x, c_y, c_z, 1.0]))
                    self.get_logger().info(f"   그리퍼 좌표계 위치: X={gripper_pos[0]:.1f} mm, Y={gripper_pos[1]:.1f} mm, Z={gripper_pos[2]:.1f} mm")
                    gx, gy, gz = gripper_pos[0], gripper_pos[1], gripper_pos[2]

                    # 접근
                    if self.approach_object(gx, gy, gz):
                       return True

            time.sleep(0.1)

        self.get_logger().error("⏱️ 탐색 타임아웃")
        return False

    def get_depth_at_point(self, depth_image, cx, cy, window=5):
        """지정 좌표의 depth 값 반환 (median 필터링)"""
        h, w = depth_image.shape
        x1 = max(0, cx - window)
        x2 = min(w, cx + window)
        y1 = max(0, cy - window)
        y2 = min(h, cy + window)

        roi = depth_image[y1:y2, x1:x2]
        valid = roi[roi > 0]

        if len(valid) > 0:
            depth_meters = np.median(valid) * self.depth_scale
            # 디버깅: depth 통계
            self.get_logger().debug(f"   Depth ROI: {len(valid)}/{roi.size} valid pixels")
            self.get_logger().debug(f"   Depth range: {valid.min()*self.depth_scale:.3f}m ~ {valid.max()*self.depth_scale:.3f}m")
            return depth_meters
        else:
            # 유효한 depth 픽셀이 없음
            self.get_logger().debug(f"   Depth ROI: 0/{roi.size} valid pixels - 모든 픽셀이 0")
            return 0.0

    def approach_object(self, offset_x, offset_y, offset_z):
        """객체에 접근 (Eye-in-Hand)"""
        self.get_logger().info(f"🚀 접근 시작")

        self.save_current_pose()

        # 1. XY 정렬 (Tool Relative)
        req = MoveLine.Request()
        req.pos = [float(offset_x-15), float(offset_y), 0.0, 0.0, 0.0, 0.0]
        req.vel = [25.0, 0.0]
        req.acc = [25.0, 0.0]
        req.ref = 1  # Tool
        req.mode = 1  # Relative
        

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        time.sleep(0.3)

        # 2. Z 접근 (Tool Relative)
        approach_dist = offset_z - 180.0  # 그리퍼 끝에서 물체 상단까지 거리 보정
        self.get_logger().info(f"   접근 거리: {approach_dist} mm (물체까지 거리: {offset_z:.1f} mm)")

        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(approach_dist), 0.0, 0.0, 0.0]
        req.vel = [20.0, 0.0]
        req.acc = [20.0, 0.0]
        req.ref = 1  # Tool
        req.mode = 1

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        self.get_logger().info("✅ 접근 완료")
        return True

    def save_current_pose(self):
        """현재 위치 저장"""
        if not self.get_pos_client.wait_for_service(timeout_sec=1.0):
            return

        req = GetCurrentPos.Request()
        req.space_type = 1  # Task Space

        future = self.get_pos_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.shaker_origin_pos = list(future.result().pos)

    def lift_object(self, height_mm):
        """객체 들어올리기 (Base Z Relative)"""
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(height_mm), 0.0, 0.0, 0.0]
        req.vel = [80.0, 0.0]  # 속도 조정 (100 → 80)
        req.acc = [60.0, 0.0]  # 가속도 조정 (100 → 60)
        req.ref = 0  # Base
        req.mode = 1  # Relative

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        return future.result() and future.result().success

    def shake_motion(self):
        """쉐이킹 모션 (흔들기)"""
        # 쉐이킹 위치 1 (속도를 낮춰서 부드럽게)
        if not self.move_to_pose(self.SHAKE_POS_1, vel=80.0, acc=60.0):
            return False
        time.sleep(0.2)

        # 쉐이킹 위치 2
        if not self.move_to_pose(self.SHAKE_POS_2, vel=80.0, acc=60.0):
            return False
        time.sleep(0.2)

        return True

    def return_shaker(self):
        """Shaker 원위치 복귀"""
        if self.shaker_origin_pos is None:
            self.get_logger().warn("저장된 위치 없음. 홈으로 복귀")
            return self.move_to_joint(self.JOINT_HOME)

        safe_pos = list(self.shaker_origin_pos)
        safe_pos[2] = 400.0  # 안전 높이
        safe_pos[1] += 80.0
        if not self.move_to_pose(safe_pos):
            return False

        # 2. 원위치로 하강
        if not self.move_to_pose(safe_pos, vel=50.0):
            return False

        return True

    def pick_cup(self, cup_type):
        """컵을 집는다"""
        if cup_type not in self.CUP_HEIGHTS:
            self.get_logger().error(f"❌ 알 수 없는 컵 종류: {cup_type}")
            return False

        cup_z = self.CUP_HEIGHTS[cup_type]
        cup_pos = [389.39, 21.52, cup_z, 10.74, -179.71, 10.58]

        self.get_logger().info(f"🥤 {cup_type} 집기 (Z={cup_z}mm)")

        # 1. 컵 위치로 이동
        if not self.move_to_pose(cup_pos):
            return False

        time.sleep(0.3)

        # 2. 그리퍼 닫기
        gripper.close_gripper()
        time.sleep(1.0)

        self.get_logger().info(f"✅ {cup_type} 집기 완료")
        return True

    def on_current_menu(self, msg):
        menu_name = msg.data
        self.get_logger().info(f"🔍 조회 메뉴==============================: {menu_name}")
        
        cup_type = self.fetch_cup_type_from_db(menu_name)
        if cup_type:
            self.cup_type = cup_type
            self.get_logger().info(f"✅ Cup Type 설정: {cup_type}")
        else:
            self.get_logger().warn(f"⚠️ DB에서 cup_type을 찾지 못했습니다. 기본값 사용: {self.cup_type}")

    def fetch_cup_type_from_db(self, menu_name):
        """DB에서 메뉴의 cup_type을 조회"""
        self.db_query_result = []
        self.db_query_event.clear()

        escaped_keyword = menu_name.replace("'", "''")
        query = f"""
        SELECT cup
        FROM bartender_menu_recipe
        WHERE menu_seq LIKE '%{escaped_keyword}%'
        ORDER BY created_at DESC
        LIMIT 0, 1
        """
        self.get_logger().info(f"🔍 DB 쿼리 전송: {query.strip()}")
        self.db_client.execute_query_with_response(query, callback=self.on_db_response)

        # 응답 대기 (최대 3초)
        self.get_logger().info("⏳ DB 응답 대기 중...")
        if self.db_query_event.wait(timeout=3.0):
            if self.db_query_result and len(self.db_query_result) > 0:
                cup_type = self.db_query_result[0].get('cup')
                return cup_type
            else:
                self.get_logger().warn(f"DB에서 '{menu_name}'의 cup 정보를 찾지 못했습니다.")
                return None
        else:
            self.get_logger().error("DB 응답 타임아웃")
            return None

    # def fetch_recipe_from_db(self, menu_seq_or_name):
    #     """DB에서 레시피 정보를 조회합니다."""
    #     self.db_query_result = []
    #     self.db_query_event.clear()

    #     escaped_keyword = menu_seq_or_name.replace("'", "''")
    #     # 요청된 쿼리문
    #     query = f"""
    #     SELECT name, pour_time, cup
    #     FROM bartender_menu_recipe
    #     WHERE menu_seq LIKE '%{escaped_keyword}%'
    #     ORDER BY created_at DESC
    #     LIMIT 0, 1
    #     """
    #     self.db_client.execute_query_with_response(query, callback=self.on_db_response)
        
    #     # 응답 대기 (최대 5초)
    #     if self.db_query_event.wait(timeout=5.0):
    #         self.get_logger().info("✅ DB 응답 수신 완료!")
    #         return self.db_query_result
    #     else:
    #         self.get_logger().error("DB Query Timeout")
    #         return []
        
    def on_db_response(self, response):
        """DB 응답 콜백"""
        if response.get('success', False):
            self.db_query_result = response.get('result', [])
            self.get_logger().info(f"✅ DB 응답 수신: {len(self.db_query_result)}개 row")
        else:
            self.get_logger().error(f"❌ DB 에러: {response.get('error')}")
            self.db_query_result = []
        self.db_query_event.set()

    # def on_db_response(self, response):
    #     if response.get('success', False):
    #         self.db_query_result = response.get('result', [])
    #     else:
    #         self.get_logger().error(f"DB Error: {response.get('error')}")
    #     self.db_query_event.set()

    def retract(self, dist_mm):
        """후퇴 (Base Z Relative)"""
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(dist_mm), 0.0, 0.0, 0.0]
        req.vel = [80.0, 0.0] 
        req.acc = [60.0, 0.0]
        req.ref = 0
        req.mode = 1

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        return future.result() and future.result().success

    def move_to_joint(self, joint_pos, vel=30.0, acc=30.0):
        """Joint 이동"""
        req = MoveJoint.Request()
        req.pos = [float(j) for j in joint_pos]
        req.vel = vel
        req.acc = acc

        future = self.move_joint_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        return future.result() and future.result().success

    def move_to_pose(self, pose, vel=30.0, acc=30.0):
        """Task Space 이동 (Absolute)"""
        req = MoveLine.Request()
        req.pos = [float(p) for p in pose]
        req.vel = [vel, 0.0]
        req.acc = [acc, 0.0]
        req.ref = 0  
        req.mode = 0 

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        return future.result() and future.result().success

    def destroy_node(self):
        self.stop_camera()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = ShakeController()
    DR_init.__dsr__node = node

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
