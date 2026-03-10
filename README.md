# SO-ARM101 자율 쓰레기 수거 로봇 캡스톤 프로젝트

> ROS2 + SO-ARM101 로봇팔을 활용한 자율 쓰레기 수거 시스템

---

## 전체 시스템 블럭도 (2학기 최종)

```mermaid
flowchart TD
    subgraph SENSOR["센서 레이어"]
        DC["뎁스카메라\n(RGB-D)"]
    end

    subgraph PERCEPTION["인식 레이어"]
        YO["YOLOv8\n쓰레기 감지"]
        PO["3D 위치 추정\n(Point Cloud)"]
    end

    subgraph DECISION["판단 레이어"]
        GD["Garbage Detector Node\n(ROS2)"]
        NAV["Navigation2\n경로 계획"]
        AC["Arm Controller Node\n(ROS2)"]
    end

    subgraph ROBOT["실행 레이어"]
        TB["TurtleBot3 Waffle\n자율주행"]
        ARM["SO-ARM101\n로봇팔 (Follower)"]
        LR["LeRobot\n모방학습 모델 (ACT)"]
    end

    subgraph OUTPUT["결과"]
        BIN["쓰레기통\n(지정 위치)"]
    end

    DC -->|"RGB + Depth"| YO
    YO -->|"BBox + Class"| PO
    PO -->|"/detected_objects\n3D 좌표"| GD

    GD -->|"/goal_pose\n이동 목표"| NAV
    GD -->|"/target_pose\n집기 목표"| AC

    NAV -->|"cmd_vel"| TB
    TB -->|"쓰레기 앞 도착"| AC

    AC --> LR
    LR -->|"관절 제어"| ARM
    ARM -->|"쓰레기 집기 완료"| NAV
    NAV -->|"쓰레기통으로 이동"| TB
    TB -->|"쓰레기통 앞 도착"| AC
    AC --> LR
    ARM -->|"쓰레기 투입"| BIN
```

---

## 1학기 블럭도

```mermaid
flowchart TD
    subgraph CAM["뎁스카메라 (RGB-D)"]
        RGB["RGB 영상"]
        DEPTH["Depth 맵"]
    end

    subgraph PC["Host PC (Ubuntu 24.04 / ROS2 Jazzy)"]
        YO["YOLOv8\n쓰레기 감지"]
        POS["3D 좌표 계산\n위치값 X, Y + 깊이값 Z"]
        AC["Arm Controller Node\n(ROS2)"]
        LR["LeRobot ACT 모델\n관절 각도 추론"]
    end

    subgraph HW["로봇팔 제어 하드웨어"]
        BOARD["SO-ARM101\n서보 제어 보드 (USB)"]
        ARM["SO-ARM101\n로봇팔 (Follower)"]
    end

    subgraph RESULT["결과"]
        GRAB["쓰레기 집기 완료"]
    end

    RGB -->|"영상 스트림"| YO
    DEPTH -->|"픽셀별 거리"| POS
    YO -->|"쓰레기 BBox"| POS
    POS -->|"3D 좌표 (X, Y, Z)"| AC
    AC -->|"모델 추론 요청"| LR
    LR -->|"관절 각도 명령"| BOARD
    BOARD -->|"서보 모터 제어"| ARM
    ARM --> GRAB
```

---

## 전체 동작 흐름

```mermaid
sequenceDiagram
    participant CAM as 뎁스카메라
    participant AI as YOLOv8
    participant GD as Garbage Detector
    participant NAV as Navigation2
    participant CAR as TurtleBot3
    participant ARM as SO-ARM101

    CAM->>AI: RGB-D 이미지 스트림
    AI->>GD: 쓰레기 감지 + 3D 위치
    GD->>NAV: 쓰레기 위치로 이동 명령
    NAV->>CAR: 경로 생성 및 이동
    CAR->>GD: 도착 완료
    GD->>ARM: 집기 동작 실행 (LeRobot ACT)
    ARM->>GD: 집기 완료
    GD->>NAV: 쓰레기통 위치로 이동 명령
    NAV->>CAR: 경로 생성 및 이동
    CAR->>GD: 도착 완료
    GD->>ARM: 투입 동작 실행 (LeRobot ACT)
```

---

## 기술 스택

| 구분 | 기술 |
|------|------|
| OS | Ubuntu 24.04 |
| 로봇 미들웨어 | ROS2 Jazzy |
| 자율주행 | TurtleBot3 Waffle + Navigation2 |
| 로봇팔 | SO-ARM101 (Leader/Follower) |
| 학습 프레임워크 | LeRobot (Hugging Face) - ACT |
| 물체 인식 | YOLOv8 + RGB-D 카메라 |
| 언어 | Python, C++ |

---

## 개발 로드맵

### 1학기 - 로봇팔 위주
- [ ] 개발 환경 구축 (ROS2 Jazzy + LeRobot)
- [ ] SO-ARM101 캘리브레이션 및 기초 제어
- [ ] ROS2 노드로 팔 제어 래핑
- [ ] 뎁스카메라 연동 및 3D 위치 추정
- [ ] 쓰레기 집기 데이터 수집 (텔레오퍼레이션)
- [ ] ACT 모델 학습 및 검증

### 2학기 - 전체 통합
- [ ] TurtleBot3 자율주행 연동
- [ ] 전체 파이프라인 통합 테스트
- [ ] 성능 최적화 및 시연
