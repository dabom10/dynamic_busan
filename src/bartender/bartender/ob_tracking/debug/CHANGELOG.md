# Object Tracking Debug Changelog

## 파일 목록

| 파일명 | 설명 | 생성일 |
|--------|------|--------|
| tracking_debug_v1.py | ROS2 없이 단독 실행 가능한 디버깅용 버전 | 2025-01-29 |

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

### 종료
- `q` 키

---

## 본 코드 (tracking.py) 변경 이력

### 2025-01-29 - 초기 버전
- PersonTracker 클래스 구현
- ROS2 노드 (PersonTrackingNode) 구현
- Publisher 추가:
  - `/person_appeared` (Bool)
  - `/person_disappeared` (Bool)
  - `/person_count` (Int32)

### 2025-01-29 - 구역 판단 기능 추가
- 화면을 3개 구역으로 분할
- 사람별 구역 판단 (bbox 중심점 기준)
- ZONE_POSITIONS 로봇 좌표 매핑
- Publisher 추가:
  - `/person_zones` (Int32MultiArray) - 각 구역별 사람 수
  - `/zone_positions` - 사람 있는 구역의 로봇 좌표