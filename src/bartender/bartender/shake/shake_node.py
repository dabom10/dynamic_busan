#!/usr/bin/env python3
"""
shake_node - 쉐이킹 제거, 컵 집기 + 고객 전달 전용

흐름: 홈 → 컵 집기 → 고객 위치 확인(tracking) → 전달 → 홈 복귀
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from bartender_interfaces.action import Motion
from bartender_interfaces.srv import DrinkDelivery
from std_msgs.msg import String
import DR_init
from bartender.onrobot import RG
from bartender.db.db_client import DBClient
import sys
import threading

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
    from dsr_msgs2.srv import SetCurrentTool
except ImportError as e:
    print(f"ERROR: dsr_msgs2 import 실패: {e}")
    print("해결방법: colcon build로 전체 워크스페이스를 빌드한 후 source install/setup.bash 실행")
    sys.exit(1)


class ShakeController(Node):
    def __init__(self):
        super().__init__("shake_node", namespace=ROBOT_ID)
        self.get_logger().info("=== Shake Node (컵 집기 + 전달 전용) ===")

        # Callback Group 생성 (Action과 DB 응답을 동시 처리)
        self._callback_group = ReentrantCallbackGroup()

        # DB 클라이언트 초기화 (callback_group 전달)
        self.db_client = DBClient(self, callback_group=self._callback_group)
        self.db_query_result = []
        self.db_query_event = threading.Event()

        # Service 클라이언트
        self.move_line_client = self.create_client(
            MoveLine, '/dsr01/motion/move_line', callback_group=self._callback_group)
        self.move_joint_client = self.create_client(
            MoveJoint, '/dsr01/motion/move_joint', callback_group=self._callback_group)
        self.set_tool_client = self.create_client(
            SetCurrentTool, '/dsr01/system/set_current_tool', callback_group=self._callback_group)

        # Tracking/Recovery 통신
        self.delivery_client = self.create_client(DrinkDelivery, 'get_pose')
        self.pub_cup_type = self.create_publisher(String, '/cup_type', 10)

        # Subscriber (current_menu 구독 → DB에서 cup_type 조회)
        self.sub_current_menu = self.create_subscription(
            String, '/current_menu', self.on_current_menu, 10)

        # Action Server 생성
        self._action_server = ActionServer(
            self,
            Motion,
            'shake/motion',
            self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self._callback_group
        )

        # 상태 변수
        self.is_running = False
        self.cup_type = "green_cup"  # 기본값

        # 컵 높이 정의
        self.CUP_HEIGHTS = {
            "green_cup": 140.0,
            "black_cup": 80.0,
            "yellow_cup": 50.0
        }

        self.JOINT_HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        self.get_logger().info("Shake Action Server ready (shake/motion)")

    def goal_callback(self, goal_request):
        """Goal 수신 여부 확인용 콜백"""
        self.get_logger().info(f"📨 goal_callback 호출됨: {goal_request.motion_name}")
        self.get_logger().info(f"   is_running={self.is_running}")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Goal 수락 후 execute_callback 디스패치"""
        self.get_logger().info("✅ handle_accepted_callback 호출됨 → execute() 시작")
        goal_handle.execute()

    def _call_sync(self, client, request, timeout=30.0):
        event = threading.Event()
        response = [None]

        future = client.call_async(request)

        def _done(fut):
            try:
                response[0] = fut.result()
            except Exception:
                pass
            event.set()

        future.add_done_callback(_done)

        if event.wait(timeout=timeout):
            return response[0]

        self.get_logger().warn(f"Service call timeout ({timeout}s)")
        return None

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Action 실행 콜백"""
        motion_name = goal_handle.request.motion_name
        self.get_logger().info(f"Goal 수신: {motion_name}")

        if self.is_running:
            self.get_logger().error(f"이미 실행 중입니다! (is_running={self.is_running})")
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

            # 2. 컵 집기
            self.publish_feedback(goal_handle, feedback_msg, 30, "컵 집기")
            if not self.pick_cup(self.cup_type):
                raise Exception("컵 집기 실패")

            # 3. 고객 위치로 전달
            self.publish_feedback(goal_handle, feedback_msg, 60, "고객 위치로 전달 중...")
            delivery_success = self.send_delivery_request()

            if not delivery_success:
                self.get_logger().warn("음료 전달 실패. 홈으로 복귀합니다.")

            # 4. 홈으로 복귀
            self.publish_feedback(goal_handle, feedback_msg, 100, "완료")
            self.move_to_joint(self.JOINT_HOME)

            elapsed_ms = int((time.time() - start_time) * 1000)
            goal_handle.succeed()

            result = Motion.Result()
            result.success = True
            result.message = f"전달 완료 '{motion_name}'"
            result.total_time_ms = elapsed_ms
            self.get_logger().info(f"전달 완료: {elapsed_ms}ms")

        except Exception as e:
            self.get_logger().error(f"실패: {e}")
            goal_handle.abort()
            result = Motion.Result()
            result.success = False
            result.message = str(e)
            result.total_time_ms = int((time.time() - start_time) * 1000)

        finally:
            self.is_running = False
            self.get_logger().info(f"is_running = {self.is_running} (초기화 완료)")

        return result

    def send_delivery_request(self):
        from DSR_ROBOT2 import movel, posx, movej

        # 서비스 연결 확인
        if not self.delivery_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("DrinkDelivery 서비스 연결 실패. 전달 건너뜀.")
            return False

        # 요청 생성
        req = DrinkDelivery.Request()
        req.finish = True  # 제작 완료 신호

        self.get_logger().info("DrinkDelivery 요청 전송...")

        cup_msg = String()
        cup_msg.data = self.cup_type
        self.pub_cup_type.publish(cup_msg)

        # 동기 호출 (threading.Event 기반, executor-safe)
        response = self._call_sync(self.delivery_client, req, timeout=10.0)

        if not response:
            self.get_logger().error("DrinkDelivery 응답 없음")
            return False

        pos = list(response.goal_position)

        self.get_logger().info(f"받은 고객 위치: {pos}")

        # 위치가 없으면 종료
        if len(pos) <= 0:
            self.get_logger().warn("반환된 좌표가 없습니다")
            return False

        try:
            # 홈 위치로 이동 (안전)
            self.get_logger().info("홈 위치로 이동")
            movej([0, 0, 90, 0, 90, 0], vel=60, acc=60)

            # 고객 위치로 이동
            self.get_logger().info(f"고객 위치로 이동: {pos}")
            movel(posx(pos), vel=60, acc=60)
            gripper.open_gripper()
            time.sleep(0.5)

            self.get_logger().info("음료 전달 완료")
            return True

        except Exception as e:
            self.get_logger().error(f"음료 전달 중 에러: {e}")
            return False

    def publish_feedback(self, goal_handle, feedback_msg, progress, step):
        feedback_msg.progress = progress
        feedback_msg.current_step = step
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f"[{progress}%] {step}")

    def pick_cup(self, cup_type):
        """컵을 집는다"""
        if cup_type not in self.CUP_HEIGHTS:
            self.get_logger().error(f"알 수 없는 컵 종류: {cup_type}")
            return False

        cup_z = self.CUP_HEIGHTS[cup_type]
        cup_pos = [389.39, 21.52, cup_z, 10.74, -179.71, 10.58]

        self.get_logger().info(f"{cup_type} 집기 (Z={cup_z}mm)")

        # 1. 컵 위치로 이동
        if not self.move_to_pose(cup_pos):
            return False

        time.sleep(0.3)

        # 2. 그리퍼 닫기
        gripper.close_gripper()
        time.sleep(1.0)

        self.get_logger().info(f"{cup_type} 집기 완료")
        return True

    def on_current_menu(self, msg):
        menu_name = msg.data
        self.get_logger().info(f"조회 메뉴: {menu_name}")

        cup_type = self.fetch_cup_type_from_db(menu_name)

        if cup_type:
            self.cup_type = cup_type
            self.get_logger().info(f"Cup Type 설정: {cup_type}")
        else:
            self.get_logger().warn(f"DB에서 cup_type을 찾지 못했습니다. 기본값 사용: {self.cup_type}")

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
        self.get_logger().info(f"DB 쿼리 전송: {query.strip()}")
        self.db_client.execute_query_with_response(query, callback=self.on_db_response)

        # 응답 대기 (최대 3초)
        self.get_logger().info("DB 응답 대기 중...")
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

    def on_db_response(self, response):
        """DB 응답 콜백"""
        if response.get('success', False):
            self.db_query_result = response.get('result', [])
            self.get_logger().info(f"DB 응답 수신: {len(self.db_query_result)}개 row")
        else:
            self.get_logger().error(f"DB 에러: {response.get('error')}")
            self.db_query_result = []
        self.db_query_event.set()

    def move_to_joint(self, joint_pos, vel=30.0, acc=30.0):
        """Joint 이동"""
        req = MoveJoint.Request()
        req.pos = [float(j) for j in joint_pos]
        req.vel = vel
        req.acc = acc

        result = self._call_sync(self.move_joint_client, req, timeout=30.0)
        return result is not None and result.success

    def move_to_pose(self, pose, vel=30.0, acc=30.0):
        """Task Space 이동 (Absolute)"""
        req = MoveLine.Request()
        req.pos = [float(p) for p in pose]
        req.vel = [vel, 0.0]
        req.acc = [acc, 0.0]
        req.ref = 0
        req.mode = 0

        result = self._call_sync(self.move_line_client, req, timeout=30.0)
        return result is not None and result.success


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
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()