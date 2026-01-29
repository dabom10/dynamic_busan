# Object Tracking Debug Changelog

## 파일 목록

| 파일명 | 설명 | 생성일 |
|--------|------|--------|
| tracking_debug_v1.py | ROS2 없이 단독 실행 가능한 디버깅용 버전 | 2025-01-29 |
| tracking_debug_v2.py | 구역 판단 기능 추가 버전 | 2025-01-29 |

---

## v1 - tracking_debug_v1.py (2025-01-29)

### 기능
- YOLOv8n + ByteTrack 사람 추적
- 새로운 사람 등장 감지 (`[DEBUG] [NEW]`)
- 사람 사라짐 감지 (`[DEBUG] [LOST]`)
- 화면에 바운딩 박스 + ID 표시
- FPS 표시

### 실행 방법
```bash
cd /home/rokey/dynamic_busan/src/bartender/bartender/ob_tracking/debug
python3 tracking_debug_v1.py
```

---

## v2 - tracking_debug_v2.py (2025-01-29)

### v1 대비 추가된 기능
- 화면 3개 구역 분할 (Zone 1, 2, 3)
- 구역별 사람 수 카운트
- ZONE_POSITIONS 로봇 좌표 매핑
- 구역별 색상 구분 시각화

### ROS2 토픽
- `/zone_status` (Int32MultiArray) - 구역별 사람 수
- `/active_zone` (Int32) - 활성 구역 번호
- `/zone_robot_pos` (Float32MultiArray) - 로봇 좌표

### 실행 방법
```bash
ros2 run bartender tracking
```

---

## 본 코드 (tracking.py) 변경 이력

### 2025-01-29 - 초기 버전
- PersonTracker 클래스 구현
- ROS2 노드 (PersonTrackingNode) 구현
- Publisher 추가:
  - `/person_appeared` (Bool)
  - `/person_disappeared` (Bool)
  - `/person_count` (Int32)

### 2025-01-29 - 구역 판단 기능 추가 (v2)
- 화면을 3개 구역으로 분할
- 사람별 구역 판단 (bbox 중심점 기준)
- ZONE_POSITIONS 로봇 좌표 매핑
- Publisher 추가:
  - `/zone_status` (Int32MultiArray)
  - `/active_zone` (Int32)
  - `/zone_robot_pos` (Float32MultiArray)

### 2025-01-29 - 바텐더 연동 기능 추가 (v3)
- `/person_disappeared` 제거
- 이름 기반 추적 기능 추가:
  - `/customer_name` (String) subscribe - 고객 이름 수신
  - 바운딩 박스에 이름 표시
- 제작 완료 연동:
  - `/make_done` (Bool) subscribe - 제작 완료 신호 수신
  - 제작 완료 시 해당 고객 구역의 로봇 좌표 publish
- 고객 사라짐 알림:
  - `/disappeared_customer_name` (String) publish - 사라진 고객 이름