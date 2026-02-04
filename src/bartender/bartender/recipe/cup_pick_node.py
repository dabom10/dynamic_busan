#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import threading
import sys
import os
import json
import time
from bartender.onrobot import RG

ROBOT_TCP = "GripperDA_v1"
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

# ==============================================================================
# [라이브러리 경로 추가]
# ==============================================================================
sys.path.append('/home/rokey/cobot_ws/install/dsr_msgs2/lib/python3.10/site-packages')
sys.path.append('/home/rokey/cobot_ws/install/dsr_msgs2/local/lib/python3.10/dist-packages')
sys.path.append('/home/rokey/cobot_ws/install/dsr_common2/local/lib/python3.10/dist-packages')

try:
    from dsr_msgs2.srv import MoveLine, MoveJoint
    from dsr_msgs2.srv import SetCtrlBoxDigitalOutput
    from dsr_msgs2.srv import SetCurrentTool
    try:
        from dsr_msgs2.srv import GetCurrentPose as GetCurrentPos
    except ImportError:
        from dsr_msgs2.srv import GetCurrentPose as GetCurrentPos
except ImportError as e:
    print(f"ERROR: dsr_msgs2 라이브러리 로드 실패: {e}")
    sys.exit(1)

class BartenderNode(Node):
    def __init__(self):
        super().__init__('bartender_cup_pick')
        self.get_logger().info("=== Bartender Bot (Fixed Version) ===")

        # 1. 파일 경로 설정
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'recipe.json')
        model_path = os.path.join(current_dir, 'best.pt')
        calib_path = os.path.join(current_dir, 'T_gripper2camera.npy')

        # 2. 데이터 로드
        if os.path.exists(json_path):
            with open(json_path, 'r', encoding='utf-8') as f:
                self.recipe_data = json.load(f)
        else:
            self.get_logger().error("recipe.json 없음"); sys.exit(1)

        self.calib_matrix = np.load(calib_path) if os.path.exists(calib_path) else np.eye(4)
        
        try:
            self.model = YOLO(model_path)
        except Exception:
            self.get_logger().error("YOLO 모델 로드 실패"); sys.exit(1)

        # 3. RealSense 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        try:
            self.profile = self.pipeline.start(config)
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.align = rs.align(rs.stream.color)
        except Exception as e:
            self.get_logger().error(f"RealSense 에러: {e}"); sys.exit(1)

        # 4. ROS 클라이언트 설정
        self.pub_img = self.create_publisher(Image, '/yolo/image', 10)
        self.br = CvBridge()
        
        self.move_line_client = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.move_joint_client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.io_client = self.create_client(SetCtrlBoxDigitalOutput, '/dsr01/io/set_ctrl_box_digital_output')
        self.get_pos_client = self.create_client(GetCurrentPos, '/dsr01/system/get_current_pos')
        self.set_tool_client = self.create_client(SetCurrentTool, '/dsr01/system/set_current_tool')

        if not self.move_line_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ 로봇 서비스 연결 실패")
        if not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⚠️ IO 서비스 연결 실패")

        # 5. 변수 초기화
        self.current_recipe = None
        self.target_object = None   
        self.task_step = "idle"     
        self.liquor_idx = 0         
        self.bottle_origin_pos = None 
        self.saved_vision_offset = [0.0, 0.0]
        self.saved_approach_dist = 0.0
        
        self.status_msg = "Waiting..."
        self.is_moving = False
        
        # [위치/높이 파라미터]
        # 컵 탐색 초기 위치 (Z=359.12)
        self.INITIAL_READY_POS = [367.47, 8.37, 359.12, 23.63, 179.98, 23.36]
        self.CURRENT_Z_HEIGHT = 359.12 

        # 컵 놓는 베이스 위치 (X, Y는 고정, Z는 가변)
        self.BASE_HOME_POS = [307.16, -12.14, 78.81, 129.37, -177.29, 139.48]
        self.JOINT_HOME_POS = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 병 탐색 위치
        self.BOTTLE_VIEW_POS = [-200.0, 600.0, 360.0, 0.0, -90.0, -90.0]

        # [추가] 병별 파라미터 (XY 보정, 접근 여유거리)
        # margin이 작을수록 병 쪽으로 더 많이 전진합니다.
        self.bottle_params = {
            "black_bottle": {"off_x": 0.0, "off_y": 0.0, "margin": 175.0},
            "blue_bottle":  {"off_x": 5.0, "off_y": 0.0, "margin": 175.0},
            "default":      {"off_x": 0.0, "off_y": 0.0, "margin": 175.0}
        }

        # ★ 컵 종류별 놓는 높이 (Z 절대 좌표)
        self.cup_place_target_z = {
            "green_cup": 150.0,
            "black_cup": 85.0,
            "yellow_cup": 55.0
        }

        self.set_robot_tcp()

        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)
        self.input_thread.start()
        self.timer = self.create_timer(0.033, self.timer_callback)

    def set_robot_tcp(self):
        if self.set_tool_client.wait_for_service(timeout_sec=1.0):
            req = SetCurrentTool.Request()
            req.name = ROBOT_TCP
            self.set_tool_client.call_async(req)

    def set_digital_output(self, index, value):
        try:
            req = SetCtrlBoxDigitalOutput.Request()
            req.index = index; req.value = value
            self.io_client.call_async(req)
        except Exception as e:
            self.get_logger().error(f"IO Error: {e}")

    def user_input_loop(self):
        time.sleep(1)
        print("\n [System] 메뉴를 입력하세요 (예: blue_sapphire)")
        while rclpy.ok():
            try:
                if self.is_moving:
                    time.sleep(1); continue
                user_input = input("\n메뉴 입력 >> ").strip()
                if not user_input: continue

                found = False
                for r in self.recipe_data.get("recipes", []):
                    if r["recipe_id"] == user_input:
                        self.current_recipe = r
                        self.target_object = r["cup"]
                        # self.task_step = "cup"
                        self.task_step = "cup"
                        self.liquor_idx = 0
                        # self.status_msg = "Moving to Start Pos..."
                        self.status_msg = "Moving to Start Pos..."
                        self.is_moving = True 
                        gripper.open_gripper()
                        self.get_logger().info(f"주문 접수: {r['display_name']}")
                        # self.move_to_initial_ready()
                        # self.move_to_initial_ready()
                        
                        # [TEST] 컵 생략하고 바로 병 작업 시작
                        self.start_bottle_sequence()
                        found = True
                        break
                if not found: print("메뉴 없음.")
            except: break

    def move_to_initial_ready(self):
        req = MoveLine.Request()
        req.pos = self.INITIAL_READY_POS
        req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
        req.ref = 0; req.mode = 0 
        future = self.move_line_client.call_async(req)
        future.add_done_callback(self.ready_to_search_cup)

    def ready_to_search_cup(self, future):
        if future.result().success:
            self.get_logger().info("초기 위치 도착. 탐색 시작.")
            self.status_msg = f"Search: {self.target_object}"
            self.is_moving = False
        else:
            self.is_moving = False

    def timer_callback(self):
        annotated_frame = None
        try:
            # 1. 프레임 받기 (타임아웃 체크)
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            if not frames:
                self.get_logger().warn("RealSense 프레임 없음!")
                return

            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame: 
                return

            # 2. 이미지 변환
            img = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # 3. YOLO 추론
            results = self.model(img, verbose=False)
            annotated_frame = results[0].plot()

            # 4. 상태 메시지 표시
            cv2.rectangle(annotated_frame, (0, 0), (640, 60), (0, 0, 0), -1)
            cv2.putText(annotated_frame, self.status_msg, (10, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 5. 객체 탐색 로직
            if not self.is_moving and self.task_step in ["cup", "bottle"] and self.target_object:
                boxes = results[0].boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    
                    if cls_name == self.target_object:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        if self.task_step == "bottle":
                            cx = (x1 + x2) // 2
                            cy = int(y1 + (y2 - y1) * 0.65)
                        else:
                            cx = (x1 + x2) // 2
                            cy = (y1 + y2) // 2 
                        
                        # 거리 측정
                        w, h = x2 - x1, y2 - y1
                        roi_x1 = max(0, int(x1 + w * 0.3)); roi_x2 = min(640, int(x2 - w * 0.3))
                        roi_y1 = max(0, int(y1 + h * 0.3)); roi_y2 = min(480, int(y2 - h * 0.3))
                        
                        # ROI 범위 예외처리
                        if roi_x1 >= roi_x2 or roi_y1 >= roi_y2:
                            continue

                        crop = depth_image[roi_y1:roi_y2, roi_x1:roi_x2]
                        valid = crop[(crop > 100) & (crop < 1200)]
                        
                        dist = 0.0
                        if len(valid) > 0:
                            dist = np.median(valid) * self.depth_scale

                        if dist > 0:
                            cv2.putText(annotated_frame, f"Dist: {dist:.3f}m", (x1, y1-20), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)

                        # 인식 범위 내 들어오면 이동 시작
                        if 0.1 < dist < 1.2:
                            cam_point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], dist)
                            c_x = cam_point[0] * 1000.0
                            c_y = cam_point[1] * 1000.0
                            c_z = cam_point[2] * 1000.0
                            
                            gripper_pos = np.dot(self.calib_matrix, np.array([c_x, c_y, c_z, 1.0]))
                            gx, gy, gz = gripper_pos[0], gripper_pos[1], gripper_pos[2]
                            
                            self.execute_eye_in_hand_move(gx, gy, gz)
                            break 

            # 6. 이미지 퍼블리시 및 출력
            try:
                msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.pub_img.publish(msg)
            except Exception as e:
                pass # 퍼블리시 에러는 무시해도 됨
            
            cv2.imshow("Bartender Vision", annotated_frame)
            if cv2.waitKey(1) == 27: rclpy.shutdown()

        except Exception as e:
            # ★★★ 여기서 진짜 원인을 출력합니다 ★★★
            self.get_logger().error(f"Vision Loop Error: {e}")
            if annotated_frame is not None:
                try:
                    self.pub_img.publish(self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))
                except: pass
                
                cv2.imshow("Bartender Vision", annotated_frame)
                if cv2.waitKey(1) == 27: rclpy.shutdown()

        except Exception: pass

    # --- 동작 로직 ---
    def execute_eye_in_hand_move(self, offset_x, offset_y, offset_z):
        self.is_moving = True
        self.bottle_approach_dist = offset_z
        self.status_msg = "Eye-in-Hand Aligning..."
        
        # 병별 파라미터 로드
        params = self.bottle_params.get(self.target_object, self.bottle_params["default"])
        
        # [수정] 병 작업 시 높이(Z) 유지 (Base Z=360)
        # Tool 좌표계 기준 Y축 이동이 높이 변화를 유발한다고 가정하고 0으로 고정
        if self.task_step == "bottle":
            self.get_logger().info(f"🍾 병 정렬: 높이 유지(Z=360)를 위해 Y축 이동 제거. 원본 Y={offset_y:.1f}")
            offset_y = 0.0
            
            # XY 오프셋 보정 적용
            offset_x += params["off_x"]
            offset_y += params["off_y"]
            self.get_logger().info(f"🔧 보정 적용({self.target_object}): X+={params['off_x']}, Y+={params['off_y']}")

        # ★ 저장: 나중에 병 놓으러 올 때 사용
        self.saved_vision_offset = [offset_x, offset_y]

        self.get_logger().info(f"🎯 XY 상대 이동: X={offset_x:.1f}, Y={offset_y:.1f}")

        # [1] XY 정렬 (그리퍼 기준 상대 이동)
        req = MoveLine.Request()
        req.pos = [float(offset_x), float(offset_y), 0.0, 0.0, 0.0, 0.0] 
        req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
        req.ref = 1; req.mode = 1 
        
        future = self.move_line_client.call_async(req)
        
        # [2] 하강 (Base 기준 절대 높이 차이 계산)
        if self.task_step == "cup":
            # 컵 잡는 높이는 보통 낮으므로(예: 85), 현재 높이에서 차이 계산
            target_pick_z = 85.0 # 잡을 때 Z (임시)
            descend_dist = target_pick_z - self.CURRENT_Z_HEIGHT 
            self.get_logger().info(f"🍺 컵 하강 준비 (Diff: {descend_dist:.1f})")
            future.add_done_callback(lambda f: self.descend_vertical(f, descend_dist))
        elif self.task_step == "bottle":
            self.get_logger().info(f"🍾 병 접근 준비 (XY 정렬 후 전진)")
            future.add_done_callback(self.approach_bottle)

    def descend_vertical(self, future, z_diff):
        if future.result().success:
            self.status_msg = "Descending..."
            
            # Base 기준(ref=0) Relative(mode=1) 이동 -> Z축 수직 하강
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, float(z_diff), 0.0, 0.0, 0.0]
            req.vel = [50.0, 0.0]; req.acc = [50.0, 0.0]
            req.ref = 0; req.mode = 1 
            
            f = self.move_line_client.call_async(req)
            if self.task_step == "bottle":
                f.add_done_callback(self.approach_bottle)
            else:
                f.add_done_callback(self.after_approach)
            f.add_done_callback(self.after_approach)
        else:
            self.get_logger().warn("XY 정렬 실패")
            self.reset_state()

    def approach_bottle(self, future=None):
        if future is None or (hasattr(future, 'result') and future.result().success):
            self.status_msg = "Approaching Bottle..."
            
            if not hasattr(self, 'bottle_approach_dist') or self.bottle_approach_dist is None:
                self.get_logger().error("⚠️ 접근 거리 정보 없음. 병 접근 실패.")
                self.reset_state()
                return

            # 병별 마진 적용하여 이동 거리 계산
            params = self.bottle_params.get(self.target_object, self.bottle_params["default"])
            margin = params["margin"]
            dist = self.bottle_approach_dist - margin
            
            # ★ 저장: 나중에 병 놓으러 올 때 사용
            self.saved_approach_dist = dist
            
            self.get_logger().info(f"🍾 병 접근 전진: {dist:.1f}mm")
            
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, float(dist), 0.0, 0.0, 0.0] # Tool Z축 전진
            req.vel = [50.0, 0.0]; req.acc = [50.0, 0.0]
            req.ref = 1; req.mode = 1 
            
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.after_approach)
        else:
            self.reset_state()

    def after_approach(self, future):
        if future.result().success:
            self.status_msg = "Gripping..."
            gripper.close_gripper()
            self.lift_object()
        else:
            self.get_logger().warn("❌ 접근(Approach) 실패 - 이동 불가")
            self.reset_state()

    def save_bottle_pos_and_lift(self, future):
        try:
            res = future.result()
            self.bottle_origin_pos = res.pos 
            self.get_logger().info(f"병 위치 저장: {self.bottle_origin_pos}")
        except Exception: pass
        self.lift_object()

    def lift_object(self):
        self.status_msg = "Lifting..."
        
        req = MoveLine.Request()
        req.pos = [0.0, 0.0, 580.0, 0.0, 0.0, 0.0]
        req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
        req.ref = 0; req.mode = 1 # Base Relative
        
        self.get_logger().info(f"🚀 병 상승: Base Z +580.0mm")
        
        future = self.move_line_client.call_async(req)
        if self.task_step == "cup":
            future.add_done_callback(self.move_to_joint_waypoint)
        else:
            future.add_done_callback(self.go_to_pour_position)

    # --- [수정된 부분] 컵 내려놓기 시퀀스 ---
    def move_to_joint_waypoint(self, future):
        if future.result().success:
            self.status_msg = "Moving to Waypoint..."
            req = MoveJoint.Request()
            req.pos = self.JOINT_HOME_POS 
            req.vel = 50.0; req.acc = 30.0
            
            # ★ 오타 수정: move_joint -> move_joint_client
            f = self.move_joint_client.call_async(req)
            f.add_done_callback(self.go_to_cup_ready_pos)
        else: self.reset_state()

    def go_to_cup_ready_pos(self, future):
        if future.result().success:
            self.status_msg = "Approaching Cup Home..."
            
            # 1. 컵 종류에 따른 목표 Z 높이 가져오기
            target_z = self.cup_place_target_z.get(self.target_object, 78.81)
            
            # 2. 안전 높이 설정 (목표 높이 + 50mm 위)
            safe_z = target_z + 50.0
            
            # 3. 목표 좌표 생성 (X, Y는 BASE_HOME 유지, Z만 변경)
            home_pos = list(self.BASE_HOME_POS)
            home_pos[2] = safe_z
            
            req = MoveLine.Request()
            req.pos = [float(x) for x in home_pos]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 0 # Base Absolute
            
            f = self.move_line_client.call_async(req)
            # 도착 후 실제 바닥으로 하강
            f.add_done_callback(lambda f: self.descend_to_place(f, target_z))
        else: self.reset_state()

    def descend_to_place(self, future, target_z):
        if future.result().success:
            self.status_msg = "Placing Cup..."
            self.get_logger().info(f"⬇️ 컵 배치 하강: Z -> {target_z}")
            
            # 절대 좌표 Z로 정밀 하강
            place_pos = list(self.BASE_HOME_POS)
            place_pos[2] = target_z
            
            req = MoveLine.Request()
            req.pos = [float(x) for x in place_pos]
            req.vel = [30.0, 0.0]; req.acc = [30.0, 0.0] # 천천히
            req.ref = 0; req.mode = 0
            
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.finish_cup_task)
        else: self.reset_state()

    def finish_cup_task(self, future):
        if future.result().success:
            self.get_logger().info("✅ 컵 배치 완료. 그리퍼 해제")
            gripper.close_gripper()
            
            # 안전하게 위로 빠져나오기 (Base 기준 +Z 100mm 상승)
            self.status_msg = "Retracting..."
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 1 # Relative
            
            f = self.move_line_client.call_async(req)
            # 상승 후 병 작업 시작
            f.add_done_callback(lambda f: threading.Timer(1.0, self.start_bottle_sequence).start())
        else: self.reset_state()

    # --- Bottle Task Chain ---
    def start_bottle_sequence(self):
        self.task_step = "bottle_transit"
        self.status_msg = "Moving to Joint Home..."
        self.get_logger().info(f"🍾 병 시퀀스 시작 (Index: {self.liquor_idx}): Joint Home 이동")
        req = MoveJoint.Request()
        req.pos = self.JOINT_HOME_POS
        req.vel = 50.0; req.acc = 30.0
        f = self.move_joint_client.call_async(req)
        f.add_done_callback(self.move_to_bottle_view)

    def move_to_bottle_view(self, future=None):
        if future is None or (hasattr(future, 'result') and future.result().success):
            self.status_msg = "Moving to Bottle View..."
            # [수정] 안전을 위해 상공(Z=580) 경유 후 하강
            self.get_logger().info(f"🍾 Bottle View 상공(Z=580)으로 이동...")
            
            high_pos = list(self.BOTTLE_VIEW_POS)
            high_pos[2] = 580.0
            
            req = MoveLine.Request()
            req.pos = [float(x) for x in high_pos]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 0
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.descend_to_bottle_view)
        else:
            self.get_logger().error("❌ 이전 이동 실패")
            self.reset_state()

    def descend_to_bottle_view(self, future):
        if future.result().success:
            self.get_logger().info(f"🍾 Bottle View 위치(Z=360)로 하강...")
            req = MoveLine.Request()
            req.pos = self.BOTTLE_VIEW_POS
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 0
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.start_bottle_search)
        else:
            self.reset_state()

    def start_bottle_search(self, future):
        if future.result().success:
            self.CURRENT_Z_HEIGHT = 360.0
            liquors = self.current_recipe.get('liquors', [])
            if self.liquor_idx < len(liquors):
                bottle_name = liquors[self.liquor_idx]['name']
                self.target_object = bottle_name
                self.task_step = "bottle"
                self.status_msg = f"Search: {bottle_name}"
                self.is_moving = False
                self.get_logger().info(f"🍾 병 찾기: {bottle_name}")
            else:
                self.get_logger().info("🍾 모든 병 처리 완료")
                self.finish_all_tasks()
        else:
            self.get_logger().error("❌ Bottle View 이동 실패")
            self.reset_state()

    def go_to_pour_position(self, future):
        if future.result().success:
            self.status_msg = "Moving to Pour..."
            # [수정] 사용자 제공 붓기 초기 좌표 (수평 상태)
            self.pour_start_pos = [400.55, -41.65, 146.83, 33.90, -174.78, 29.70]
            self.get_logger().info(f"🍷 붓기 위치로 이동 시작 (1. 상공 이동)")

            # [수정] 안전한 이동을 위해: 상공(Z=580)으로 먼저 수평 이동 후 하강
            high_pour_pos = list(self.pour_start_pos)
            high_pour_pos[2] = 580.0 # 병을 들어올린 높이 유지
            
            req = MoveLine.Request()
            req.pos = [float(x) for x in high_pour_pos]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 0
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.descend_to_pour)
        else: self.reset_state()

    def descend_to_pour(self, future):
        if future.result().success:
            self.get_logger().info("🍷 붓기 높이로 하강 (2. 수직 하강)")
            req = MoveLine.Request()
            req.pos = [float(x) for x in self.pour_start_pos]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 0
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.pour_action)
        else: self.reset_state()

    def pour_action(self, future):
        if future.result().success:
            self.status_msg = "Pouring..."
            self.get_logger().info("🍷 따르기 (기울이기)")
            
            # [추가] pour_time 가져오기
            try:
                pour_time = float(self.current_recipe['liquors'][self.liquor_idx].get('pour_time', 2.0))
            except (IndexError, KeyError, TypeError, ValueError):
                pour_time = 2.0

            # [수정] 사용자 제공 완전히 붓는 좌표 (수직 상태)
            pour_end_pos = [429.46, -18.07, 168.70, 112.55, -140.10, 67.13]
            
            req = MoveLine.Request()
            req.pos = pour_end_pos
            req.vel = [60.0, 0.0]; req.acc = [60.0, 0.0]
            req.time = pour_time # [수정] pour_time 동안 이동
            req.ref = 0; req.mode = 0 
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.wait_and_return)
        else: self.reset_state()

    def wait_and_return(self, future):
        if future.result().success:
            # [수정] 이미 pour_time 동안 이동했으므로 추가 대기는 짧게 설정
            self.get_logger().info(f"⏳ 붓기 완료. 복귀 준비")
            time.sleep(0.5)
            
            self.status_msg = "Returning..."
            # 다시 초기 위치(수평)로 복귀하여 병 세우기
            req = MoveLine.Request()
            req.pos = self.pour_start_pos
            req.vel = [60.0, 0.0]; req.acc = [60.0, 0.0]
            req.ref = 0; req.mode = 0
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.place_bottle_back)
        else: self.reset_state()

    def place_bottle_back(self, future):
        if future.result().success:
            self.status_msg = "Returning Bottle..."
            self.get_logger().info("🍾 병 원래 위치로 복귀 시작 (1. 수직 상승)")
            
            # [수정] 먼저 수직으로 상승하여 안전 높이 확보 (Base Relative Z+350)
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, 350.0, 0.0, 0.0, 0.0]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 1 # Relative
            
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.move_to_bottle_origin_high)
        else: self.reset_state()

    def move_to_bottle_origin_high(self, future):
        if future.result().success:
            self.get_logger().info("🍾 상공으로 이동 (2. 수평 이동)")
            # BOTTLE_VIEW_POS 상공 (Z=580)으로 이동
            high_pos = list(self.BOTTLE_VIEW_POS)
            high_pos[2] = 580.0
            
            req = MoveLine.Request()
            req.pos = [float(x) for x in high_pos]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 0; req.mode = 0
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.place_bottle_align_high)
        else: self.reset_state()

    def place_bottle_align_high(self, future):
        if future.result().success:
            # 3. Vision Offset 적용 (Tool Relative) - 상공에서 수행
            off_x, off_y = self.saved_vision_offset
            self.get_logger().info(f"🍾 상공 위치 보정: X={off_x:.1f}, Y={off_y:.1f}")
            req = MoveLine.Request()
            req.pos = [float(off_x), float(off_y), 0.0, 0.0, 0.0, 0.0]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 1; req.mode = 1
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.place_bottle_approach_high)
        else: self.reset_state()

    def place_bottle_approach_high(self, future):
        if future.result().success:
            # 4. 상공에서 접근 (Approach) - XY 이동
            dist = self.saved_approach_dist
            self.get_logger().info(f"🍾 상공 접근 (XY 이동): {dist:.1f}mm")
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, float(dist), 0.0, 0.0, 0.0]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 1; req.mode = 1 # Tool Relative
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.descend_to_place)
        else: self.reset_state()

    def descend_to_place(self, future):
        if future.result().success:
            # 5. 수직 하강 (Z=580 -> 360)
            self.get_logger().info("🍾 수직 하강 (Z=580 -> 360)")
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, -220.0, 0.0, 0.0, 0.0] # Base Relative Z down
            req.vel = [50.0, 0.0]; req.acc = [50.0, 0.0]
            req.ref = 0; req.mode = 1 # Relative
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.release_bottle)
        else: self.reset_state()

    def release_bottle(self, future):
        if future.result().success:
            self.get_logger().info("🍾 병 놓기 (Release)")
            gripper.open_gripper()
            time.sleep(0.5)
            
            # 4. 후진 (Retract) - 접근했던 거리만큼 뒤로
            dist = -self.saved_approach_dist
            self.get_logger().info(f"🍾 후진: {dist:.1f}mm")
            
            req = MoveLine.Request()
            req.pos = [0.0, 0.0, float(dist), 0.0, 0.0, 0.0]
            req.vel = [100.0, 0.0]; req.acc = [100.0, 0.0]
            req.ref = 1; req.mode = 1
            f = self.move_line_client.call_async(req)
            f.add_done_callback(self.next_bottle)

    def next_bottle(self, future):
        self.liquor_idx += 1
        self.get_logger().info(f"🍾 다음 병 준비 (Index: {self.liquor_idx})")
        self.move_to_bottle_view(future)

    def finish_all_tasks(self):
        self.status_msg = "All Done. Homing..."
        
        # [수정] 작업 종료 시 상태를 초기화하여 비전 루프가 다시 도는 것을 방지 (바닥 충돌 해결)
        self.task_step = "idle"
        self.target_object = None
        
        req = MoveJoint.Request()
        req.pos = self.JOINT_HOME_POS
        req.vel = 50.0; req.acc = 30.0
        self.move_joint_client.call_async(req)
        self.reset_state()

    def reset_state(self):
        time.sleep(1.0)
        self.is_moving = False
        self.status_msg = "Ready"
        if self.task_step == "idle": self.current_recipe = None

    def destroy_node(self):
        try: self.pipeline.stop()
        except: pass
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    import DR_init
    DR_init.__dsr__id = "dsr01"
    DR_init.__dsr__model = "m0609"

    node = BartenderNode()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import get_tcp

    if get_tcp() != ROBOT_TCP:
        print(f"엔드이펙터 - Gripper 오류: {get_tcp()} != {ROBOT_TCP}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()