# simbongsa — Hey Doopal 시각장애인 보조 로봇

두산로보틱스 로키 부트캠프 **협동 프로젝트 2 · D-2조** 제출 코드입니다.

음성 웨이크워드 **"Hey Doopal"** 로 호출하면, 두산 협동로봇(M0609)이 카메라로
책상 위 물체와 사용자의 손을 인식하고, 지정한 물건을 집어 사용자의 손 위치까지
가져다주는 것을 목표로 하는 ROS 2 워크스페이스입니다. 관리자용 웹 대시보드(Flask +
Redis)로 인식된 객체·웨이포인트·대화 로그를 조회할 수 있습니다.

---

## 시스템 구성

```
  ┌──────────────┐   음성/웨이크워드    ┌──────────────────┐
  │  vla /       │────────────────────▶│  robot_control   │
  │  voice_      │  VoiceKeyword.srv   │  (M0609 + 그리퍼) │
  │  processing  │◀────────────────────│                  │
  └──────┬───────┘   /say (TTS)        └────────┬─────────┘
         │                                      │ FindOrder.action
         │                                      │ ScanRequest / GripBoundingBox
         ▼                                      ▼
  ┌──────────────┐                     ┌──────────────────┐
  │  OpenAI      │                     │ assistive_       │
  │  Whisper /   │                     │ detection        │
  │  LangChain   │                     │ (손·물체 인식,   │
  └──────────────┘                     │  RealSense+YOLO) │
                                       └────────┬─────────┘
                                                │ Redis Bridge (토픽/서비스)
                                                ▼
                                       ┌──────────────────┐
                                       │  ui_db           │
                                       │  Flask UI +      │
                                       │  Redis + Bridge  │
                                       └──────────────────┘

  공용 인터페이스: hey_doopal_msg (srv/action), vla/od_msg (SrvDepthPosition)
```

---

## 패키지 목록

| 폴더 | ROS 2 패키지 | 빌드 타입 | 역할 |
|---|---|---|---|
| [`hey_doopal_msg/`](hey_doopal_msg) | `hey_doopal_msg` | `ament_cmake` | 프로젝트 공용 srv 7종 · action 2종 정의 |
| [`assistive_detection/`](assistive_detection) | `assistive_detection` | `ament_python` | RealSense + MediaPipe 손 인식, YOLO 물체 인식, 3D 좌표 변환 |
| [`robot_control/`](robot_control) | `robot_control` | `ament_python` | 두산 M0609 제어, Modbus 그리퍼, 콘 스캔 동작 |
| [`ui_db/`](ui_db) | (ROS 패키지 아님) | Flask 앱 | 관리자 웹 대시보드 + Redis 저장소 + ROS 2 ↔ Redis Bridge |
| [`vla/od_msg/`](vla/od_msg) | `od_msg` | `ament_cmake` | `SrvDepthPosition.srv` (타겟명 → depth 좌표) |
| [`vla/voice_processing/`](vla/voice_processing) | `voice_processing` | `ament_python` | 웨이크워드 감지, STT(Whisper), LLM 의도 분석, TTS |

> `ui_db/`, `vla/od_msg/`, `vla/voice_processing/` 는 각각 별도 디렉터리로 묶여 있으므로,
> colcon 워크스페이스의 `src/` 아래에 배치할 때는 각 패키지 폴더(`package.xml` 이 있는
> 위치)를 심볼릭 링크하거나 복사해서 사용하세요.

---

## 요구 환경

- **ROS 2 Humble** (Ubuntu 22.04)
- **Python 3.10**
- 워크스페이스에 함께 있어야 하는 외부 패키지 (이 저장소에는 미포함)
  - `dsr_msgs2`, 두산 ROS 2 드라이버(`DR_init`, `DSR_ROBOT2`) — `robot_control`, `assistive_detection`
- 하드웨어
  - Intel RealSense 뎁스 카메라 (RGB / aligned depth / camera_info 발행)
  - 두산 협동로봇 M0609 (`dsr01`) + Modbus 그리퍼 (`192.168.1.1:502`)
  - 마이크 (기본 `device_index=4`, 48 kHz — `voice_processing/MicController.py`에서 조정)
- 외부 서비스
  - `voice_processing` 는 **OpenAI API** 사용 (Whisper STT, LangChain 의도 분석). `OPENAI_API_KEY` 필요

### Python 의존성

```bash
# 인식
pip install -r assistive_detection/requirements.txt        # mediapipe, ultralytics

# 음성 (voice_processing)
pip install pyaudio openai langchain langchain-openai python-dotenv \
            gtts openwakeword scipy numpy

# 로봇 제어 (robot_control)
pip install pymodbus

# 웹 대시보드 (ui_db)
pip install -r ui_db/requirements.txt                      # Flask, redis, python-dotenv
```

---

## 빌드

```bash
# 워크스페이스 예시: ~/ws_cobot_pjt/ws_dsr
cd ~/ws_cobot_pjt/ws_dsr/src

# 이 저장소를 받아 각 패키지를 src 아래로 배치
git clone https://github.com/rkdals5903-lgtm/simbongsa.git
ln -s simbongsa/hey_doopal_msg          hey_doopal_msg
ln -s simbongsa/assistive_detection     assistive_detection
ln -s simbongsa/robot_control           robot_control
ln -s simbongsa/vla/od_msg              od_msg
ln -s simbongsa/vla/voice_processing/voice_processing  voice_processing

cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

인터페이스 패키지만 먼저 빌드하려면:

```bash
colcon build --symlink-install --packages-select hey_doopal_msg od_msg
```

---

## 실행

### 1. 인식 노드 (`assistive_detection`)

RealSense 와 두산 로봇 bringup 이 먼저 떠 있어야 합니다.

```bash
ros2 launch assistive_detection detection.launch.py
# 또는 개별 실행
ros2 run assistive_detection hand_detection     # /find_hand_order (FindOrder.action)
ros2 run assistive_detection hand_tracking      # 이미지 서보 SpeedL 손 추종
ros2 run assistive_detection object_detection   # yolo_scan_request / grip_bounding_box / find_target_order
```

파라미터: [`assistive_detection/config/hand_nodes.yaml`](assistive_detection/config/hand_nodes.yaml)
(base_frame, calibration_frame, target_depth_mm, 속도 한계 등)

동봉 데이터 파일(파이썬 패키지 안에 설치됨):
- `assistive_detection/assistive_detection/T_gripper2camera.npy` — Hand-Eye 캘리브레이션 행렬 (link_6 기준)
- `assistive_detection/assistive_detection/my_seg_best.pt` — 학습된 YOLO 세그멘테이션 모델 (약 20 MB)

### 2. 음성 처리 (`voice_processing`)

`resource/.env` 에 API 키를 넣습니다 (저장소에는 미포함):

```bash
echo "OPENAI_API_KEY=sk-..." > vla/voice_processing/voice_processing/resource/.env
```

```bash
ros2 run voice_processing get_keyword   # 웨이크워드 → STT → 의도 분석 → VoiceKeyword.srv 호출
ros2 run voice_processing tts           # /say (std_msgs/String) 구독 → gTTS 재생, BEEP:: / SCAN:: 효과음
```

웨이크워드 모델: `resource/hey_doopal_final.tflite` (openWakeWord, threshold 0.5)

### 3. 로봇 제어 (`robot_control`)

```bash
python3 -m robot_control.robot_control     # 상태 머신: 음성 요청 → 스캔 → 그립 → 손 위치로 전달
```

- 그리퍼: Modbus TCP `192.168.1.1:502`
- 로봇: `dsr01` / `m0609`, VELOCITY 500 / ACC 60
- `cone_scan.py` 의 `ConeScanner` 는 대상 주변을 원뿔형으로 틸트 스캔하며 `FindOrder` 액션을 반복 호출

### 4. 웹 대시보드 + Redis Bridge (`ui_db`)

```bash
cd ui_db
cp .env.example .env          # 값 채우기 (아래 "설정" 참고)

# 원커맨드 스크립트: Redis 컨테이너 + Flask + ROS Bridge 동시 실행
./start_ui_db.sh              # http://<LAN-IP>:5000
./logs_ui_db.sh
./stop_ui_db.sh
```

수동 실행:

```bash
docker compose up -d redis                 # docker-compose.yml (redis:7.4-alpine, requirepass)
python3 app.py                             # Flask UI (기본 0.0.0.0:5000)
ros2 run <설치위치> ros_object_bridge       # ROS 2 ↔ Redis Bridge 노드
```

구성 요소:
- `app.py` — Flask 앱, `/` 사용자 화면, `/admin` 관리자(로그인 필요), `/api/*`
- `redis_store.py` — 객체 / 고정 웨이포인트 / 스캔 CASE / 대화 로그 저장소, 고정 데이터 버전 마이그레이션
- `ros_object_bridge.py` — 인식 결과를 Redis 에 저장, `GetDbData` / `GetFixedPose` / `GetObjectPose` / `GetScanCase` 서비스 제공, `/rosout` 필터링 런타임 로그
- `runtime_log.py` — Redis 에 넣지 않는 관리자 런타임 로그 파일 (`runtime_logs.jsonl`, 회전)

---

## `hey_doopal_msg` 인터페이스

### 서비스

| 파일 | 요청 → 응답 요약 |
|---|---|
| `srv/VoiceKeyword.srv` | `target, goal` → `accepted` — 음성 명령을 로봇 제어에 전달 |
| `srv/ScanRequest.srv` | `waypoint_id` → `success, message, detected_count` — 웨이포인트에서 1프레임 스캔 후 DB 저장 |
| `srv/GripBoundingBox.srv` | `target` → `coordinate[3], bbox_width/height, camera_depth_z, grip_angle_deg, is_find` — 그립 직전 재확인 |
| `srv/GetDbData.srv` | `data_type, name` → `success, json_data, message` — 범용 DB 조회 |
| `srv/GetObjectPose.srv` | `object_name` → `success, has_pose, pose[6], 단위/프레임, json_data` |
| `srv/GetFixedPose.srv` | `pose_name` → `success, pose[6], 단위/프레임, json_data` — 고정 웨이포인트 |
| `srv/GetScanCase.srv` | `case_name` → `success, pose_name_1..3, pose_1..3[6], 단위/프레임` |

### 액션

| 파일 | Goal → Result / Feedback |
|---|---|
| `action/FindOrder.action` | `target_name` → `found, coordinate[3], message` / `state` |
| `action/OptimizeGripPose.action` | `target` → `success, message` / `is_find, bbox_width, bbox_height, camera_depth_z` |

### 메시지

| 파일 | 내용 |
|---|---|
| `msg/TargetPoint.msg` | `x, y, z` (base_link 기준, mm) |

`vla/od_msg/srv/SrvDepthPosition.srv`: `target` → `depth_position[]`

---

## 설정 (`ui_db/.env`)

`ui_db/.env.example` 을 복사해 사용합니다. 주요 항목:

| 키 | 설명 |
|---|---|
| `FLASK_SECRET_KEY` | Flask 세션 서명 키 (긴 랜덤 문자열) |
| `ADMIN_USERNAME` / `ADMIN_PASSWORD` | 관리자 로그인 |
| `REDIS_HOST` / `PORT` / `DB` / `USERNAME` / `PASSWORD` / `SSL` | Redis 접속. `docker-compose.yml` 이 `REDIS_PASSWORD` 로 `requirepass` 설정 |
| `SEED_DEMO_OBJECTS` | 데모 객체 시드 여부 |
| `RUNTIME_LOG_FILE` / `MAX_BYTES` / `BACKUP_COUNT` | 관리자 런타임 로그 파일 회전 |
| `ROSOUT_LOG_FILTER` | `/rosout` 에서 관리자 로그로 남길 노드명/키워드 |

> **`.env` 는 커밋하지 마세요.** 저장소에는 `.env.example` 만 포함되어 있고,
> `.gitignore` 가 `.env` · `.venv/` · `__pycache__/` · 런타임 로그를 제외합니다.

---

## 알려진 이슈 / 정리 필요

- `robot_control/setup.py` 의 `console_scripts` 가 실제 파일과 불일치합니다
  (`final_robot_control`, `test2_robot_control` → 존재하지 않는 모듈).
  현재는 `python3 -m robot_control.robot_control` 로 실행하거나 엔트리포인트를
  `robot_control.robot_control:main` 등으로 수정해야 합니다.
- `voice_processing/setup.py` 의 `control_server`, `ui_test` 엔트리포인트도
  저장소에 없는 모듈(`robot_control_server`, `ui_chat_test`)을 가리킵니다.
  동작하는 것은 `get_keyword`, `tts` 입니다.
- `robot_control`, `od_msg`, `voice_processing` 의 `package.xml` 라이선스/설명이
  템플릿 기본값(`TODO`)입니다.
- 이 저장소는 원본 `sonnanlo2125-a11y/simbongsa` 의 fork 를 독립 저장소로 전환한
  것으로, colcon 워크스페이스가 아니라 패키지 소스 모음입니다.

---

## 팀

두산로보틱스 로키 · 협동 프로젝트 2 · **D-2조** — 김현우, 박현정, 서강민, 장동일
