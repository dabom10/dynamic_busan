import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import os

# [설정] 로봇 기동 파라미터 및 홈 위치
VELJ = 60.0  
ACCJ = 60.0  
J_READY = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

class FailureRecoveryNode(Node):
    """
    객체 인식 실패 정보와 제조 완료 신호를 결합하여
    보관 공정을 수행하는 백그라운드 상시 대기 노드
    """
    def __init__(self):
        super().__init__('failure_recovery_node')
        
        # 데이터 관리 변수
        self.mission_status = "IDLE"
        self.last_failed_customer = "미확인 고객" # 인식 실패 토픽으로 업데이트됨
        self.current_customer = None
        
        # 이동 좌표 설정
        self.storage_pose = {"x": 1.5, "y": 0.5, "w": 1.0}
        self.home_joint = J_READY
        
        # [발행자] 로봇 목적지 전송
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # [구독자 1] 객체 인식 실패 이름 수신 (비전 노드로부터)
        self.sub_disappeared = self.create_subscription(
            String, 
            '/disappeared_customer_name', 
            self.disappeared_callback, 
            10)
        
        # [구독자 2] 제조 완료 신호 수신 (제조 노드로부터)
        self.sub_made = self.create_subscription(
            String, 
            '/manufacturing_done', 
            self.start_recovery_mission, 
            10)

        # 시퀀스 제어용 타이머 (0.1초 간격)
        self.timer = self.create_timer(0.1, self.state_machine_callback)
        self.start_time = None
        
        self.get_logger().info('='*50)
        self.get_logger().info("🚀 복구 시스템 가동: 인식 실패 & 제조 완료 대기 중")
        self.get_logger().info('='*50)

    def disappeared_callback(self, msg):
        """인식 실패 신호가 오면 이름을 변수에 저장해 둡니다."""
        self.last_failed_customer = msg.data.strip()
        self.get_logger().warn(f"⚠️ 인식 실패 접수: [{self.last_failed_customer}] (제조 완료 시 즉시 이동)")

    def start_recovery_mission(self, msg):
        """제조 완료 신호가 오면 저장된 이름을 사용하여 미션을 시작합니다."""
        if self.mission_status != "IDLE":
            return

        # 제조 완료 토픽에 이름이 있으면 사용하고, 없으면 미리 저장된 이름을 사용
        msg_name = msg.data.strip()
        self.current_customer = msg_name if msg_name else self.last_failed_customer
        
        self.get_logger().error(f"🚨 [미션 시작] {self.current_customer}님의 음료를 보관대로 이동!")
        
        # 공정 시작
        self.mission_status = "MOVING_TO_STORAGE"
        self.send_goal(self.storage_pose, "보관대")
        self.start_time = self.get_clock().now()

    def state_machine_callback(self):
        """전체 공정 시퀀스 제어 루프"""
        if self.mission_status == "IDLE" or self.start_time is None:
            return

        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.mission_status == "MOVING_TO_STORAGE" and elapsed_time > 3.0:
            self.get_logger().info(f"📥 [{self.current_customer}] 하역 단계 진입")
            self.mission_status = "UNLOADING"
            self.start_time = self.get_clock().now()

        elif self.mission_status == "UNLOADING" and elapsed_time > 2.0:
            self.get_logger().info("✅ 하역 완료. 홈 복귀 명령.")
            self.send_joint_home()
            self.mission_status = "MOVING_TO_HOME"
            self.start_time = self.get_clock().now()

        elif self.mission_status == "MOVING_TO_HOME" and elapsed_time > 3.0:
            self.get_logger().info(f"🏁 [{self.current_customer}]님 보관 완료. 다시 대기 모드.")
            self.get_logger().info("-" * 50)
            
            # 초기화 및 다음 미션 대기
            self.mission_status = "IDLE"
            self.current_customer = None
            self.start_time = None

    def send_goal(self, pose_data, label):
        """이동 좌표 발행"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = pose_data["x"]
        msg.pose.position.y = pose_data["y"]
        msg.pose.orientation.w = pose_data["w"]
        self.goal_pub.publish(msg)
        self.get_logger().info(f"🚚 {label} 이동 (속도:{VELJ})")

    def send_joint_home(self):
        """홈 조인트 발행"""
        joints = self.home_joint
        msg = PoseStamped()
        msg.pose.position.x = float(joints[0])
        msg.pose.position.y = float(joints[1])
        msg.pose.position.z = float(joints[2])
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FailureRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()