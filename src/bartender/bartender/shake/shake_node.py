#!/usr/bin/env python3
"""
Shake Node - YOLO 객체인식 + 로봇 그립 + 쉐이킹 모션
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bartender_interfaces.action import Motion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import DR_init
from bartender.onrobot import RG
import os
import sys
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "GripperDA_v1"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

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
        self.get_logger().info("=== Shake Node (Vision + Motion) ===")

        # 파일 경로 설정
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'shake.pt')
        # 캘리브레이션 파일은 recipe 폴더에서 참조
        recipe_dir = os.path.join(os.path.dirname(current_dir), 'recipe')
        calib_path = os.path.join(recipe_dir, 'T_gripper2camera.npy')

        # 캘리브레이션 매트릭스 로드
        if os.path.exists(calib_path):
            self.calib_matrix = np.load(calib_path)
            self.get_logger().info(f"캘리브레이션 로드: {calib_path}")
        else:
            self.calib_matrix = np.eye(4)
            self.get_logger().warn("캘리브레이션 파일 없음. 단위 행렬 사용.")

        # YOLO 모델 로드
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"YOLO 모델 로드: {model_path}")
        except Exception as e:
            self.get_logger().error(f"YOLO 모델 로드 실패: {e}")
            sys.exit(1)

        # RealSense (필요할 때만 열고 닫음)
        self.pipeline = None
        self.align = None
        self.depth_scale = None

        # ROS 퍼블리셔/클라이언트
        self.pub_img = self.create_publisher(Image, '/shake/yolo_image', 10)
        self.br = CvBridge()

        self.move_line_client = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.move_joint_client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.get_pos_client = self.create_client(GetCurrentPos, '/dsr01/system/get_current_pose')
        self.set_tool_client = self.create_client(SetCurrentTool, '/dsr01/system/set_current_tool')

        # 서비스 대기
        if not self.move_line_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ move_line 서비스 연결 실패")
        if not self.move_joint_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ move_joint 서비스 연결 실패")

        # Action Server 생성
        self._action_server = ActionServer(
            self,
            Motion,
            'shake/motion',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # 상태 변수
        self.is_running = False
        self.target_object = "shaker"
        self.status_msg = "Ready"
        self.shaker_origin_pos = None

        # 위치 파라미터 (TODO: 실제 환경에 맞게 조정)
        self.JOINT_HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.SEARCH_POS = [300.0, 0.0, 400.0, 0.0, 180.0, 0.0]  # shaker 탐색 위치
        self.SHAKE_POS_1 = [350.0, 0.0, 300.0, 45.0, 135.0, 45.0]  # 쉐이킹 자세 1
        self.SHAKE_POS_2 = [350.0, 0.0, 300.0, -45.0, -135.0, -45.0]  # 쉐이킹 자세 2

        # TCP 설정
        self.set_robot_tcp()

        self.get_logger().info("Shake Action Server ready (shake/motion)")

    def start_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("RealSense 시작")

    def stop_camera(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
            self.align = None
            self.get_logger().info("RealSense 종료")

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
            # 1. 홈 위치로 이동
            self.publish_feedback(goal_handle, feedback_msg, 10, "홈 위치 이동")
            if not self.move_to_joint(self.JOINT_HOME):
                raise Exception("홈 위치 이동 실패")

            # 2. 탐색 위치로 이동
            self.publish_feedback(goal_handle, feedback_msg, 20, "탐색 위치 이동")
            if not self.move_to_pose(self.SEARCH_POS):
                raise Exception("탐색 위치 이동 실패")

            # 3. 객체 탐색 및 그립
            self.publish_feedback(goal_handle, feedback_msg, 30, "Shaker 탐색 중...")
            self.start_camera()
            gripper.open_gripper()
            time.sleep(0.5)

            detection_result = self.detect_and_approach()
            self.stop_camera()

            if not detection_result:
                raise Exception("Shaker 탐색 실패")

            # 4. 그립
            self.publish_feedback(goal_handle, feedback_msg, 50, "Shaker 그립")
            gripper.close_gripper()
            time.sleep(1.0)

            # 5. 들어올리기
            self.publish_feedback(goal_handle, feedback_msg, 55, "들어올리기")
            if not self.lift_object(150.0):
                raise Exception("들어올리기 실패")

            # 6. 쉐이킹 모션
            self.publish_feedback(goal_handle, feedback_msg, 60, "쉐이킹 시작")
            shake_count = 3  # 쉐이킹 횟수
            for i in range(shake_count):
                progress = 60 + int((i + 1) / shake_count * 25)
                self.publish_feedback(goal_handle, feedback_msg, progress, f"쉐이킹 {i+1}/{shake_count}")
                if not self.shake_motion():
                    raise Exception(f"쉐이킹 {i+1} 실패")

            # 7. 원위치 복귀
            self.publish_feedback(goal_handle, feedback_msg, 90, "원위치 복귀")
            if not self.return_shaker():
                raise Exception("원위치 복귀 실패")

            # 8. 그리퍼 해제 및 후퇴
            self.publish_feedback(goal_handle, feedback_msg, 95, "그리퍼 해제")
            gripper.open_gripper()
            time.sleep(0.5)
            self.retract(100.0)

            # 9. 홈으로 복귀
            self.publish_feedback(goal_handle, feedback_msg, 100, "완료")
            self.move_to_joint(self.JOINT_HOME)

            elapsed_ms = int((time.time() - start_time) * 1000)
            goal_handle.succeed()

            result = Motion.Result()
            result.success = True
            result.message = f"Shake motion '{motion_name}' 완료"
            result.total_time_ms = elapsed_ms
            self.get_logger().info(f"✅ 쉐이킹 완료: {elapsed_ms}ms")

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

    def publish_feedback(self, goal_handle, feedback_msg, progress, step):
        feedback_msg.progress = progress
        feedback_msg.current_step = step
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f"📢 [{progress}%] {step}")

    def detect_and_approach(self, timeout=10.0):
        """객체 탐색 및 접근"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            # 프레임 획득
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            if not frames:
                continue

            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # YOLO 추론
            results = self.model(img, verbose=False)
            annotated_frame = results[0].plot()

            # 상태 표시
            cv2.rectangle(annotated_frame, (0, 0), (640, 40), (0, 0, 0), -1)
            cv2.putText(annotated_frame, f"Searching: {self.target_object}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 이미지 퍼블리시
            try:
                msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.pub_img.publish(msg)
            except:
                pass

            # 객체 탐지
            boxes = results[0].boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]

                if cls_name == self.target_object:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # Depth 측정
                    dist = self.get_depth_at_point(depth_image, cx, cy)

                    if 0.1 < dist < 1.0:
                        self.get_logger().info(f"🎯 {cls_name} 발견! 거리: {dist:.3f}m")

                        # 3D 좌표 계산
                        cam_point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)
                        c_x = cam_point[0] * 1000.0
                        c_y = cam_point[1] * 1000.0
                        c_z = cam_point[2] * 1000.0

                        # 그리퍼 좌표로 변환
                        gripper_pos = np.dot(self.calib_matrix, np.array([c_x, c_y, c_z, 1.0]))
                        gx, gy, gz = gripper_pos[0], gripper_pos[1], gripper_pos[2]

                        self.get_logger().info(f"📍 Tool 좌표: [{gx:.1f}, {gy:.1f}, {gz:.1f}]")

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
            return np.median(valid) * self.depth_scale
        return 0.0

    def approach_object(self, offset_x, offset_y, offset_z):
        """객체에 접근 (Eye-in-Hand)"""
        self.get_logger().info(f"🚀 접근: X={offset_x:.1f}, Y={offset_y:.1f}, Z={offset_z:.1f}")

        # 현재 위치 저장 (복귀용)
        self.save_current_pose()

        # 1. XY 정렬 (Tool Relative)
        req = MoveLine.Request()
        req.pos = [float(offset_x), float(offset_y), 0.0, 0.0, 0.0, 0.0]
        req.vel = [100.0, 0.0]
        req.acc = [100.0, 0.0]
        req.ref = 1  # Tool
        req.mode = 1  # Relative

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.result() or not future.result().success:
            self.get_logger().error("XY 정렬 실패")
            return False

        time.sleep(0.3)

        # 2. Z 접근 (Tool Relative) - 여유 거리 50mm
        approach_dist = offset_z - 50.0
        if approach_dist < 50.0:
            approach_dist = 50.0

        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(approach_dist), 0.0, 0.0, 0.0]
        req.vel = [50.0, 0.0]
        req.acc = [50.0, 0.0]
        req.ref = 1
        req.mode = 1

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.result() or not future.result().success:
            self.get_logger().error("Z 접근 실패")
            return False

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
            self.get_logger().info(f"💾 위치 저장: {self.shaker_origin_pos[:3]}")

    def lift_object(self, height_mm):
        """객체 들어올리기 (Base Z Relative)"""
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(height_mm), 0.0, 0.0, 0.0]
        req.vel = [100.0, 0.0]
        req.acc = [100.0, 0.0]
        req.ref = 0  # Base
        req.mode = 1  # Relative

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        return future.result() and future.result().success

    def shake_motion(self):
        """쉐이킹 모션 (흔들기)"""
        # 쉐이킹 위치 1
        if not self.move_to_pose(self.SHAKE_POS_1, vel=150.0):
            return False
        time.sleep(0.2)

        # 쉐이킹 위치 2
        if not self.move_to_pose(self.SHAKE_POS_2, vel=150.0):
            return False
        time.sleep(0.2)

        return True

    def return_shaker(self):
        """Shaker 원위치 복귀"""
        if self.shaker_origin_pos is None:
            self.get_logger().warn("저장된 위치 없음. 홈으로 복귀")
            return self.move_to_joint(self.JOINT_HOME)

        # 1. 안전 높이로 이동
        safe_pos = list(self.shaker_origin_pos)
        safe_pos[2] = 400.0  # 안전 높이

        if not self.move_to_pose(safe_pos):
            return False

        # 2. 원위치로 하강
        if not self.move_to_pose(self.shaker_origin_pos, vel=50.0):
            return False

        return True

    def retract(self, dist_mm):
        """후퇴 (Base Z Relative)"""
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, float(dist_mm), 0.0, 0.0, 0.0]
        req.vel = [100.0, 0.0]
        req.acc = [100.0, 0.0]
        req.ref = 0
        req.mode = 1

        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        return future.result() and future.result().success

    def move_to_joint(self, joint_pos, vel=60.0, acc=40.0):
        """Joint 이동"""
        req = MoveJoint.Request()
        req.pos = [float(j) for j in joint_pos]
        req.vel = vel
        req.acc = acc

        future = self.move_joint_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        return future.result() and future.result().success

    def move_to_pose(self, pose, vel=100.0, acc=100.0):
        """Task Space 이동 (Absolute)"""
        req = MoveLine.Request()
        req.pos = [float(p) for p in pose]
        req.vel = [vel, 0.0]
        req.acc = [acc, 0.0]
        req.ref = 0  # Base
        req.mode = 0  # Absolute

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

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
