# dynamic_busan

gantt
    title Bartender Robot Project Schedule
    dateFormat  YYYY-MM-DD
    axisFormat  %m/%d

    section 공통
    주제 확정 및 구조 설계 시작      :done, 2026-01-28, 1d
    GitHub 페이지 생성               :done, 2026-01-28, 1d
    물품 구입                        :done, 2026-01-29, 2d
    중간 발표                        :milestone, 2026-01-30, 0d
    JSON → DB 전환                   :active, 2026-02-04, 1d
    통합 흐름 개선                   :active, 2026-02-05, 1d

    section 다봄 (Cup Pick / Vision)
    Cup Pick 초안 완성               :done, 2026-01-29, 1d
    모델 분석                        :done, 2026-01-30, 1d
    Baseline 모델 선정 및 생성       :done, 2026-01-31, 1d
    Cup Pick 기본 동작 구현          :done, 2026-02-01, 1d
    새로운 모델 적용                :done, 2026-02-02, 1d
    Cup Pick 동작 통합               :done, 2026-02-03, 1d
    JSON → DB 연동 변경              :active, 2026-02-04, 1d
    Cup Pick 에러 디버깅             :active, 2026-02-05, 1d

    section 성호 (Tracking / Supervisor)
    Tracking 초안                    :done, 2026-01-28, 1d
    Tracking 1차 완성 + Pub          :done, 2026-01-29, 1d
    Tracking 보완 + 멀티스레딩       :done, 2026-01-30, 1d
    Tracking·Topping 통신 테스트     :done, 2026-01-31, 1d
    Tracking 서비스 통신 완성        :done, 2026-02-01, 1d
    Recovery 구조 개선               :done, 2026-02-02, 1d
    Supervisor·STT 구조 개선         :done, 2026-02-03, 1d
    Supervisor-Tracking 통신 구현    :done, 2026-02-04, 1d
    STT 개선 및 통합 흐름 완성       :active, 2026-02-05, 1d

    section 동찬 (DB / Supervisor)
    DB 노드 설계 및 구현             :done, 2026-01-29, 1d
    Supervisor 노드 설계 및 구현     :done, 2026-01-30, 1d
    컨디션 이슈                     :crit, 2026-02-01, 4d

    section 도영 (STT / Service)
    음성 인식 설계 및 구현           :done, 2026-01-28, 3d
    트래킹·토핑 서비스 통신 구현     :done, 2026-01-31, 1d
    컨디션 이슈                     :crit, 2026-02-01, 4d
    음료 추천 기능 추가              :active, 2026-02-05, 1d

    section 정호 (Recovery / Topping)
    Recovery 설계 및 구현            :done, 2026-01-28, 3d
    컨디션 이슈                     :crit, 2026-02-02, 3d
    Topping 기능 구현                :active, 2026-02-05, 1d
