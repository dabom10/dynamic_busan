"""
tracking.py - ROS2 사람 추적 노드

웹캠(고정)에서 YOLOv8n + ByteTrack을 사용하여 사람을 추적하고,
등장/사라짐 이벤트를 ROS2 토픽으로 publish합니다.

======================================================================
토픽 목록:
----------------------------------------------------------------------
    /person_appeared      (std_msgs/Bool)  - 새 사람 등장 시 True
    /person_disappeared   (std_msgs/Bool)  - 사람 사라짐 시 True
    /person_count         (std_msgs/Int32) - 현재 추적 중인 사람 수
======================================================================

실행 방법:
    ros2 run bartender tracking_node

디버깅/테스트:
    tracking_debug_v1.py 파일 사용 (ROS2 없이 단독 실행 가능)
"""

import time
import cv2
from ultralytics import YOLO

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


# =============================================================================
# PersonTracker 클래스
# -----------------------------------------------------------------------------
# YOLOv8 + ByteTrack 기반 사람 추적기
#
# 주요 기능:
#   1. 사람 객체 탐지 (YOLOv8n, class 0 = person)
#   2. 객체 추적 및 ID 부여 (ByteTrack)
#   3. 새로운 사람 등장 감지
#   4. 사람 사라짐 감지 (N프레임 이상 미탐지 시)
#
# 사용 예시:
#   tracker = PersonTracker(conf=0.35, lost_threshold=30)
#   tracks, events = tracker.update(frame)
#   # events['new'] = 새로 등장한 ID 리스트
#   # events['lost'] = 사라진 ID 리스트
# =============================================================================
class PersonTracker:
    """YOLOv8 + ByteTrack 기반 사람 추적기 (고정 웹캠용)"""

    def __init__(self, model_path='yolov8n.pt', conf=0.35, lost_threshold=30):
        """
        Args:
            model_path: YOLOv8 모델 경로 (기본값: yolov8n.pt)
            conf: 탐지 신뢰도 임계값 (0.0 ~ 1.0, 기본값: 0.35)
            lost_threshold: 사라짐 판정 프레임 수 (기본 30프레임 ≈ 1초 @30fps)
        """
        self.model = YOLO(model_path)
        self.conf = conf
        self.lost_threshold = lost_threshold

        # ---------------------------------------------------------------------
        # 추적 상태 관리 변수
        # ---------------------------------------------------------------------
        # tracked_persons: 현재 추적 중인 사람들의 정보
        #   - key: track_id (정수)
        #   - value: {'last_seen': 마지막 탐지 프레임, 'bbox': (x1,y1,x2,y2)}
        self.tracked_persons = {}

        # frame_count: 처리한 총 프레임 수
        self.frame_count = 0

        # disappeared_persons: 이번 프레임에서 사라진 것으로 판정된 ID 리스트
        self.disappeared_persons = []

        # new_persons: 이번 프레임에서 새로 등장한 ID 리스트
        self.new_persons = []

    def update(self, frame):
        """프레임을 처리하고 추적 결과를 반환

        Args:
            frame: BGR 이미지 (numpy array, OpenCV 형식)

        Returns:
            tracks: 추적 결과 리스트 [(track_id, (x1,y1,x2,y2), confidence), ...]
            events: 이벤트 딕셔너리 {'new': [새 ID들], 'lost': [사라진 ID들]}
        """
        self.frame_count += 1
        self.disappeared_persons = []
        self.new_persons = []

        # ---------------------------------------------------------------------
        # YOLOv8 + ByteTrack 추적 실행
        # ---------------------------------------------------------------------
        # persist=True: 프레임 간 트래킹 ID 유지
        # tracker="bytetrack.yaml": ByteTrack 알고리즘 사용
        # classes=[0]: person 클래스만 탐지
        results = self.model.track(
            frame,
            persist=True,
            tracker="bytetrack.yaml",
            conf=self.conf,
            classes=[0],
            verbose=False
        )[0]

        current_ids = set()
        tracks = []

        # ---------------------------------------------------------------------
        # 탐지 결과 처리
        # ---------------------------------------------------------------------
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes

            # track ID가 있는 경우만 처리 (ByteTrack이 할당한 ID)
            if boxes.id is not None:
                for i, track_id in enumerate(boxes.id.int().tolist()):
                    bbox = boxes.xyxy[i].int().tolist()
                    conf = boxes.conf[i].item()

                    x1, y1, x2, y2 = bbox
                    current_ids.add(track_id)
                    tracks.append((track_id, (x1, y1, x2, y2), conf))

                    # 새로운 사람 등장 체크
                    if track_id not in self.tracked_persons:
                        self.new_persons.append(track_id)
                        print(f"[NEW] 새로운 사람 등장: ID {track_id}")

                    # 추적 정보 업데이트
                    self.tracked_persons[track_id] = {
                        'last_seen': self.frame_count,
                        'bbox': (x1, y1, x2, y2)
                    }

        # ---------------------------------------------------------------------
        # 사라진 사람 확인
        # ---------------------------------------------------------------------
        # lost_threshold 프레임 이상 미탐지 시 사라진 것으로 판정
        for track_id, info in list(self.tracked_persons.items()):
            frames_missing = self.frame_count - info['last_seen']

            if frames_missing > self.lost_threshold:
                self.disappeared_persons.append(track_id)
                print(f"[LOST] 사람 사라짐: ID {track_id} ({frames_missing}프레임 미탐지)")
                del self.tracked_persons[track_id]

        events = {
            'new': self.new_persons,
            'lost': self.disappeared_persons
        }

        return tracks, events

    def get_active_count(self):
        """현재 추적 중인 사람 수 반환"""
        return len(self.tracked_persons)


# =============================================================================
# 시각화 함수
# =============================================================================
def draw_results(frame, tracks, events, fps):
    """추적 결과를 프레임에 시각화

    Args:
        frame: BGR 이미지
        tracks: [(track_id, bbox, conf), ...]
        events: {'new': [ids], 'lost': [ids]}
        fps: 현재 FPS

    Returns:
        frame: 시각화가 추가된 프레임
    """
    for track_id, bbox, conf in tracks:
        x1, y1, x2, y2 = bbox

        # 새로 등장한 사람: 파란색 / 기존 사람: 초록색
        if track_id in events['new']:
            color = (255, 100, 0)  # 파란색 (BGR)
            label = f'NEW ID {track_id}'
        else:
            color = (0, 255, 0)  # 초록색 (BGR)
            label = f'ID {track_id} ({conf:.2f})'

        # 바운딩 박스 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # 라벨 배경 + 텍스트
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
        cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # 사라진 사람 알림 (빨간색)
    if events['lost']:
        lost_text = f"LOST: ID {', '.join(map(str, events['lost']))}"
        cv2.putText(frame, lost_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # 상단 정보 표시
    cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(frame, f'Tracking: {len(tracks)} persons', (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    return frame


# =============================================================================
# ROS2 노드 클래스
# =============================================================================
class PersonTrackingNode(Node):
    """ROS2 사람 추적 노드

    웹캠에서 사람을 추적하고 등장/사라짐 이벤트를 publish합니다.

    Published Topics:
        /person_appeared (std_msgs/Bool): 새 사람 등장 시 True
        /person_disappeared (std_msgs/Bool): 사람 사라짐 시 True
        /person_count (std_msgs/Int32): 현재 추적 중인 사람 수
    """

    def __init__(self):
        super().__init__('person_tracking_node')

        # ---------------------------------------------------------------------
        # ROS2 Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('lost_threshold', 30)
        self.declare_parameter('show_window', True)

        self.camera_id = self.get_parameter('camera_id').value
        self.confidence = self.get_parameter('confidence').value
        self.lost_threshold = self.get_parameter('lost_threshold').value
        self.show_window = self.get_parameter('show_window').value

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        # 사람 등장 이벤트 (True = 1, 새 사람 등장)
        self.pub_appeared = self.create_publisher(Bool, '/person_appeared', 10)

        # 사람 사라짐 이벤트 (True = 1, 사람 사라짐)
        self.pub_disappeared = self.create_publisher(Bool, '/person_disappeared', 10)

        # 현재 추적 중인 사람 수
        self.pub_count = self.create_publisher(Int32, '/person_count', 10)
        # ---------------------------------------------------------------------
        # 웹캠 초기화
        # ---------------------------------------------------------------------
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'웹캠을 열 수 없습니다: {self.camera_id}')
            raise RuntimeError(f'웹캠을 열 수 없습니다: {self.camera_id}')

        # C270 HD 웹캠 해상도 설정 (720p)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'웹캠 해상도: {actual_w}x{actual_h}')

        # ---------------------------------------------------------------------
        # 트래커 초기화
        # ---------------------------------------------------------------------
        self.tracker = PersonTracker(
            conf=self.confidence,
            lost_threshold=self.lost_threshold
        )

        # FPS 계산용
        self.fps_time = time.time()

        # 타이머로 프레임 처리 (약 30fps)
        self.timer = self.create_timer(0.033, self.process_frame)

        self.get_logger().info('='*50)
        self.get_logger().info('사람 추적 노드 시작')
        self.get_logger().info('='*50)

    def process_frame(self):
        """프레임 처리 및 이벤트 publish"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임을 읽을 수 없습니다.')
            return

        # 추적 수행
        tracks, events = self.tracker.update(frame)

        # FPS 계산
        current_time = time.time()
        fps = 1.0 / (current_time - self.fps_time + 1e-6)
        self.fps_time = current_time

        # -----------------------------------------------------------------
        # ROS2 토픽 Publish
        # -----------------------------------------------------------------
        # 사람 등장 이벤트
        if events['new']:
            msg = Bool()
            msg.data = True  # 1 = 등장
            self.pub_appeared.publish(msg)
            self.get_logger().info(f'[PUB] person_appeared = True (ID: {events["new"]})')

        # 사람 사라짐 이벤트
        if events['lost']:
            msg = Bool()
            msg.data = True  # 1 = 사라짐
            self.pub_disappeared.publish(msg)
            self.get_logger().info(f'[PUB] person_disappeared = True (ID: {events["lost"]})')

        # 현재 추적 인원 수
        count_msg = Int32()
        count_msg.data = self.tracker.get_active_count()
        self.pub_count.publish(count_msg)

        # 시각화
        if self.show_window:
            frame = draw_results(frame, tracks, events, fps)
            cv2.imshow('Person Tracker (ROS2)', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('사용자에 의해 종료됨')
                self.destroy_node()
                rclpy.shutdown()

    def destroy_node(self):
        """노드 종료 시 리소스 정리"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


# =============================================================================
# 메인 함수
# =============================================================================
def main(args=None):
    rclpy.init(args=args)

    try:
        node = PersonTrackingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'에러 발생: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()