import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from srv_interfaces.srv import DrinkDelivery
from geometry_msgs.msg import Point
import DR_init
import cv2
import numpy as np
from ultralytics import YOLO
import pyrealsense2 as rs2

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v2"

class ToppingNode(Node):
    def __init__(self):
        super().__init__("topping_node", namespace=ROBOT_ID)
        
        # YOLO 모델 로드
        self.model = YOLO('yolov8s.pt')  # 또는 'yolov8n.pt'
        self.bridge = CvBridge()
        
        # RealSense 카메라 구독
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        
        # 서비스 클라이언트
        self.delivery_client = self.create_client(DrinkDelivery, 'get_pose')
        
        # 데이터 저장
        self.color_image = None
        self.depth_image = None
        self.camera_intrinsics = None
        
        # 토핑 클래스 매핑 (YOLO 학습 시 사용한 클래스명)
        self.topping_classes = {
            'white_duck': 0,
            'yellow_duck': 1,
            'leaf': 2
        }
        
        # 주요 위치 정의 (임의 좌표로, 수정 필요)
        self.TOPPING_STATION = [300, 0, 400, 0, 180, 0]  # 토핑 인식 위치
        self.DRINK_STATION = [250, 100, 350, 0, 180, 0]  # 음료 위치
        self.HOME_POSITION = [0, 0, 90, 0, 90, 0]
        
        # 카메라-로봇 변환 행렬 (캘리브레이션 필요)
        self.camera_to_robot_transform = np.eye(4)
        
        self.get_logger().info("토핑 노드 초기화 완료")

    def camera_info_callback(self, msg):
        """카메라 내부 파라미터 저장"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = rs2.intrinsics()
            self.camera_intrinsics.width = msg.width
            self.camera_intrinsics.height = msg.height
            self.camera_intrinsics.ppx = msg.k[2]
            self.camera_intrinsics.ppy = msg.k[5]
            self.camera_intrinsics.fx = msg.k[0]
            self.camera_intrinsics.fy = msg.k[4]
            self.camera_intrinsics.model = rs2.distortion.brown_conraddy
            self.camera_intrinsics.coeffs = list(msg.d)

    def color_callback(self, msg):
        """컬러 이미지 수신"""
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        """뎁스 이미지 수신"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def detect_topping(self, target_topping):
        """
        YOLO로 특정 토핑 감지
        :param target_topping: 'white_duck', 'yellow_duck', 'leaf'
        :return: (x, y, depth) 또는 None
        """
        if self.color_image is None or self.depth_image is None:
            self.get_logger().warn("카메라 이미지 없음")
            return None
        
        # YOLO 추론
        results = self.model(self.color_image)
        
        target_class_id = self.topping_classes.get(target_topping)
        if target_class_id is None:
            self.get_logger().error(f"알 수 없는 토핑: {target_topping}")
            return None
        
        # 결과 파싱
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                
                if cls == target_class_id and conf > 0.5:
                    # 바운딩 박스 중심점
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    # 깊이 값 추출 (주변 픽셀 평균)
                    depth_roi = self.depth_image[
                        max(0, center_y-5):min(self.depth_image.shape[0], center_y+5),
                        max(0, center_x-5):min(self.depth_image.shape[1], center_x+5)
                    ]
                    depth = np.median(depth_roi[depth_roi > 0])
                    
                    self.get_logger().info(
                        f"{target_topping} 감지: pixel({center_x}, {center_y}), depth={depth}mm"
                    )
                    return (center_x, center_y, depth)
        
        self.get_logger().warn(f"{target_topping} 미감지")
        return None

    def pixel_to_robot_coords(self, pixel_x, pixel_y, depth):
        """
        픽셀 좌표 + 깊이 -> 로봇 좌표 변환
        :return: [x, y, z, rx, ry, rz]
        """
        if self.camera_intrinsics is None:
            self.get_logger().error("카메라 파라미터 없음")
            return None
        
        # 픽셀 -> 3D 카메라 좌표
        depth_m = depth / 1000.0  # mm to m
        point_3d = rs2.rs2_deproject_pixel_to_point(
            self.camera_intrinsics, [pixel_x, pixel_y], depth_m
        )
        
        # 카메라 좌표계 -> 로봇 좌표계 변환
        camera_point = np.array([point_3d[0], point_3d[1], point_3d[2], 1.0])
        robot_point = self.camera_to_robot_transform @ camera_point
        
        # 로봇 포즈 형식으로 반환 (x, y, z, rx, ry, rz)
        # 회전값은 그리퍼 방향에 따라 조정
        robot_pose = [
            robot_point[0] * 1000,  # m to mm
            robot_point[1] * 1000,
            robot_point[2] * 1000,
            0, 180, 0  # 고정 회전값 (실제로는 조정 필요)
        ]
        
        return robot_pose

    def grasp_topping(self, topping_pose):
        """토핑 그리핑"""
        from DSR_ROBOT2 import movel, movej, posx
        from dsr_control2 import set_digital_output
        
        # 접근 위치 (토핑 위 50mm)
        approach_pose = topping_pose.copy()
        approach_pose[2] += 50
        
        self.get_logger().info("토핑 접근")
        movel(posx(approach_pose), vel=50, acc=50)
        
        # 그리퍼 열기
        set_digital_output(1, 0)  # 그리퍼 포트 확인 필요
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # 하강
        self.get_logger().info("토핑 그립")
        movel(posx(topping_pose), vel=30, acc=30)
        
        # 그리퍼 닫기
        set_digital_output(1, 1)
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # 상승
        movel(posx(approach_pose), vel=50, acc=50)

    def place_topping_on_drink(self):
        """음료 위에 토핑 올리기"""
        from DSR_ROBOT2 import movel, posx
        from dsr_control2 import set_digital_output
        
        # 음료 위치로 이동
        self.get_logger().info("음료 위치로 이동")
        drink_pose = self.DRINK_STATION.copy()
        drink_pose[2] += 100  # 음료 위 100mm
        
        movel(posx(drink_pose), vel=50, acc=50)
        
        # 토핑 올리기 위치로 하강
        drink_pose[2] -= 50
        movel(posx(drink_pose), vel=30, acc=30)
        
        # 그리퍼 열기
        set_digital_output(1, 0)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # 상승
        drink_pose[2] += 100
        movel(posx(drink_pose), vel=50, acc=50)

    def deliver_drink(self):
        """손님에게 음료 배달"""
        from DSR_ROBOT2 import movel, movej, posx
        from dsr_control2 import set_digital_output
        
        # 서비스로 손님 위치 받기
        while not self.delivery_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("배달 서비스 대기 중...")
        
        request = DrinkDelivery.Request()
        request.finish = True
        future = self.delivery_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        customer_pose = list(response.goal_position)
        
        if len(customer_pose) < 6:
            self.get_logger().error("유효하지 않은 손님 위치")
            return
        
        self.get_logger().info(f"손님 위치: {customer_pose}")
        
        # 음료 그립
        self.get_logger().info("음료 그립")
        drink_grasp_pose = self.DRINK_STATION.copy()
        drink_grasp_pose[2] -= 20  # 음료 높이에 맞게 조정
        
        movel(posx(drink_grasp_pose), vel=50, acc=50)
        set_digital_output(1, 1)  # 그리퍼 닫기
        rclpy.spin_once(self, timeout_sec=1.0)
        
        # 음료 들어올리기
        drink_grasp_pose[2] += 100
        movel(posx(drink_grasp_pose), vel=50, acc=50)
        
        # 손님 위치로 이동
        self.get_logger().info("손님에게 배달")
        movel(posx(customer_pose), vel=60, acc=60)
        
        # 음료 내려놓기
        set_digital_output(1, 0)  # 그리퍼 열기
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # 홈으로 복귀
        movej(self.HOME_POSITION, vel=60, acc=60)
        
        self.get_logger().info("배달 완료")

    def execute_topping_task(self, recipe_topping):
        """
        전체 토핑 작업 실행
        :param recipe_topping: 'white_duck', 'yellow_duck', 'leaf'
        """
        from DSR_ROBOT2 import movej
        
        try:
            # 1. 홈 위치로
            self.get_logger().info("홈 위치로 이동")
            movej(self.HOME_POSITION, vel=60, acc=60)
            
            # 2. 토핑 스테이션으로 이동
            self.get_logger().info("토핑 스테이션으로 이동")
            movej(self.TOPPING_STATION, vel=60, acc=60)
            rclpy.spin_once(self, timeout_sec=1.0)  # 카메라 안정화
            
            # 3. 토핑 감지
            self.get_logger().info(f"{recipe_topping} 감지 시도")
            detection = self.detect_topping(recipe_topping)
            
            if detection is None:
                self.get_logger().error("토핑 감지 실패")
                return False
            
            # 4. 좌표 변환
            pixel_x, pixel_y, depth = detection
            topping_pose = self.pixel_to_robot_coords(pixel_x, pixel_y, depth)
            
            if topping_pose is None:
                self.get_logger().error("좌표 변환 실패")
                return False
            
            self.get_logger().info(f"토핑 로봇 좌표: {topping_pose}")
            
            # 5. 토핑 그립
            self.grasp_topping(topping_pose)
            
            # 6. 음료에 토핑 올리기
            self.place_topping_on_drink()
            
            # 7. 음료 배달
            self.deliver_drink()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"작업 실패: {str(e)}")
            return False


def main():
    rclpy.init()
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = ToppingNode()
    DR_init.__dsr__node = node

    try:
        # 레시피에 따른 토핑 선택 (파라미터나 서비스로 받을 수도 있음)
        recipe_topping = 'white_duck'  # 예시
        
        # 토핑 작업 실행
        success = node.execute_topping_task(recipe_topping)
        
        if success:
            node.get_logger().info("모든 작업 완료!")
        else:
            node.get_logger().error("작업 실패")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()