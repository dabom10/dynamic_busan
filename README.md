# dynamic_busan
gantt
    title Bartender Robot Project Gantt Chart
    dateFormat  YYYY-MM-DD
    axisFormat  %m/%d

    %% =========================
    %% 스타일 정의 (발표용 컬러)
    %% =========================
    classDef common fill:#E3F2FD,stroke:#1565C0,stroke-width:2px,color:#0D47A1;
    classDef dabom fill:#FCE4EC,stroke:#C2185B,stroke-width:2px,color:#880E4F;
    classDef seongho fill:#E8F5E9,stroke:#2E7D32,stroke-width:2px,color:#1B5E20;
    classDef dongchan fill:#FFF3E0,stroke:#EF6C00,stroke-width:2px,color:#E65100;
    classDef doyung fill:#F3E5F5,stroke:#6A1B9A,stroke-width:2px,color:#4A148C;
    classDef jungho fill:#E0F2F1,stroke:#00695C,stroke-width:2px,color:#004D40;
    classDef issue fill:#ECEFF1,stroke:#455A64,stroke-dasharray: 5 5,color:#263238;

    %% =========================
    section 공통 일정
    주제 확정 및 구조 설계 시작      :done, c1, 2026-01-28, 1d :::common
    GitHub 페이지 생성               :done, c2, 2026-01-28, 1d :::common
    물품 구입                        :done, c3, 2026-01-29, 2d :::common
    중간 발표                        :milestone, mid, 2026-01-30, 0d
    JSON → DB 전환                   :done, c4, 2026-02-04, 1d :::common
    통합 흐름 개선                   :done, c5, 2026-02-05, 1d :::common

    %% =========================
    section 다봄 (Cup Pick / Vision)
    Cup Pick 초안 완성               :done, d1, 2026-01-29, 1d :::dabom
    모델 분석                        :done, d2, 2026-01-30, 1d :::dabom
    Baseline 모델 선정 및 생성       :done, d3, 2026-01-31, 1d :::dabom
    Cup Pick 기본 동작 구현          :done, d4, 2026-02-01, 1d :::dabom
    새로운 모델 적용                :done, d5, 2026-02-02, 1d :::dabom
    Cup Pick 동작 통합               :done, d6, 2026-02-03, 1d :::dabom
    JSON → DB 연동 변경              :done, d7, 2026-02-04, 1d :::dabom
    Cup Pick 에러 디버깅             :done, d8, 2026-02-05, 1d :::dabom

    %% =========================
    section 성호 (Tracking / Supervisor / Recovery)
    Tracking 초안                    :done, s1, 2026-01-28, 1d :::seongho
    Tracking 1차 완성 + Pub          :done, s2, 2026-01-29, 1d :::seongho
    Tracking 보완 + Recovery MT      :done, s3, 2026-01-30, 1d :::seongho
    Tracking·Topping 통신 테스트     :done, s4, 2026-01-31, 1d :::seongho
    Tracking 서비스 통신 완성        :done, s5, 2026-02-01, 1d :::seongho
    Recovery 컵 종류 Sub 추가        :done, s6, 2026-02-02, 1d :::seongho
    Supervisor·STT 구조 개선         :done, s7, 2026-02-03, 1d :::seongho
    Supervisor-Tracking 통신 구현    :done, s8, 2026-02-04, 1d :::seongho
    STT 개선 및 통합 흐름 개선       :done, s9, 2026-02-05, 1d :::seongho

    %% =========================
    section 동찬 (DB / Supervisor)
    DB 노드 설계 및 구현             :done, dc1, 2026-01-29, 1d :::dongchan
    Supervisor 노드 설계 및 구현     :done, dc2, 2026-01-30, 1d :::dongchan
    컨디션 이슈                     :crit, dc3, 2026-02-01, 4d :::issue

    %% =========================
    section 도영 (STT / 서비스)
    음성 인식 설계 및 구현           :done, dy1, 2026-01-28, 3d :::doyung
    트래킹·토핑 서비스 통신 구현     :done, dy2, 2026-01-31, 1d :::doyung
    컨디션 이슈                     :crit, dy3, 2026-02-01, 4d :::issue
    음료 추천 기능 추가              :done, dy4, 2026-02-05, 1d :::doyung

    %% =========================
    section 정호 (Recovery / Topping)
    Recovery 설계 및 구현            :done, j1, 2026-01-28, 3d :::jungho
    컨디션 이슈                     :crit, j2, 2026-02-02, 3d :::issue
    Topping 기능 구현                :done, j3, 2026-02-05, 1d :::jungho
