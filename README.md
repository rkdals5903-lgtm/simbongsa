## hey_doopal

ROS 2 기반의 시각장애인 보조 로봇 프로젝트입니다.
음성 웨이크워드 **"Hey Doopal"** 로 로봇을 호출하면, 두산 협동로봇(M0609)이 카메라로
책상 위 물체와 사용자의 손을 인식하고, 사용자가 말한 물건을 집어 손 위치까지 가져다줍니다.
관리자용 웹 대시보드(Flask + Redis)로 인식된 객체·웨이포인트·대화 로그를 조회할 수 있습니다.

두산로보틱스 로키 부트캠프 **협동 프로젝트 2 · D-2조** 제출 코드입니다.

## 프로젝트 개요

`hey_doopal`은 음성 명령 기반 물체 전달을 수행하는 ROS 2 패키지 모음입니다.

주요 처리 흐름은 다음과 같습니다.

1. 마이크 입력에서 웨이크워드 "Hey Doopal" 감지
2. 발화를 녹음해 STT(OpenAI Whisper)로 텍스트 변환
3. LLM(LangChain)으로 대상 물체와 동작 의도 분석
4. `VoiceKeyword` 서비스로 로봇 제어 노드에 명령 전달
5. 로봇이 책상을 스캔(`ScanRequest`)해 물체 좌표를 DB에 저장
6. 대상 물체 좌표를 조회하고 `GripBoundingBox` / `FindOrder`로 정밀 위치 확인
7. 물체를 집은 뒤 `FindOrder`(hand)로 사용자 손을 찾아 이동
8. 손 추종(`hand_tracking`, SpeedL)으로 손 앞까지 접근 후 물체 전달
9. 전 과정의 상태·객체·대화 로그를 Redis에 저장하고 웹 UI로 표시

## 주요 기능

* openWakeWord 기반 웨이크워드 감지 (`hey_doopal_final.tflite`)
* OpenAI Whisper STT + LangChain 의도 분석
* gTTS 기반 음성 피드백(TTS) 및 효과음 재생
* RealSense + MediaPipe 손바닥 3D 좌표 추정
* YOLO 세그멘테이션(`my_seg_best.pt`) 물체 인식 및 3D 좌표 변환
* Hand-Eye 캘리브레이션(`T_gripper2camera.npy`) 기반 카메라→로봇 베이스 좌표 변환
* 두산 M0609 제어, Modbus 그리퍼 제어, 콘 스캔 동작
* 이미지 서보(SpeedL) 방식의 손 추종
* ROS 2 service / action 기반 노드 간 연동
* ROS 2 ↔ Redis Bridge 및 Flask 관리자 대시보드
* `/rosout` 필터링 런타임 로그 수집

## 패키지 구성

```text
simbongsa/
├── README.md
├── .gitignore
├── hey_doopal_msg/            (ament_cmake) 프로젝트 공용 인터페이스
│   ├── srv/                   VoiceKeyword, ScanRequest, GripBoundingBox,
│   │                          GetDbData, GetObjectPose, GetFixedPose, GetScanCase
│   ├── action/                FindOrder, OptimizeGripPose
│   ├── msg/                   TargetPoint
│   ├── CMakeLists.txt
│   └── package.xml
├── assistive_detection/       (ament_python) 손·물체 인식
│   ├── assistive_detection/
│   │   ├── hand_detection.py
│   │   ├── hand_tracking.py
│   │   ├── object_detection.py
│   │   ├── T_gripper2camera.npy
│   │   └── my_seg_best.pt
│   ├── config/hand_nodes.yaml
│   ├── launch/detection.launch.py
│   ├── package.xml / setup.py / setup.cfg
│   └── README.md
├── robot_control/             (ament_python) 두산 M0609 제어
│   └── robot_control/
│       ├── robot_control.py   상태 머신 메인 노드
│       └── cone_scan.py       ConeScanner (원뿔형 틸트 스캔)
├── ui_db/                     (Flask 앱) 관리자 대시보드 + Redis Bridge
│   ├── app.py                 Flask UI (/, /admin, /api/*)
│   ├── redis_store.py         객체/웨이포인트/대화 로그 저장소
│   ├── ros_object_bridge.py   ROS 2 ↔ Redis Bridge 노드
│   ├── runtime_log.py         관리자 런타임 로그 파일(회전)
│   ├── docker-compose.yml     redis:7.4-alpine
│   ├── start_ui_db.sh / stop_ui_db.sh / logs_ui_db.sh
│   ├── static/ templates/
│   └── .env.example
└── vla/
    ├── od_msg/                (ament_cmake) SrvDepthPosition.srv
    └── voice_processing/
        └── voice_processing/  (ament_python) 음성 처리
            ├── get_keyword.py     웨이크워드→STT→의도분석 노드
            ├── tts_node.py        /say 구독 → 음성 재생 노드
            ├── wakeup_word.py     openWakeWord 래퍼
            ├── stt.py             Whisper STT 래퍼
            ├── MicController.py   PyAudio 마이크 스트림
            └── resource/          beep.mp3, drum.mp3, hey_doopal_final.tflite
```

> `ui_db/`, `vla/od_msg/`, `vla/voice_processing/`는 각각 별도 디렉터리로 묶여 있습니다.
> colcon 워크스페이스의 `src/` 아래에는 `package.xml`이 있는 각 패키지 폴더를 배치합니다.

## 설치 방법

ROS 2 워크스페이스를 생성합니다.

```bash
mkdir -p ~/ws_cobot_pjt/ws_dsr/src
cd ~/ws_cobot_pjt/ws_dsr/src
```

저장소를 clone합니다.

```bash
git clone https://github.com/rkdals5903-lgtm/simbongsa.git
```

각 패키지를 `src` 아래로 연결합니다.

```bash
ln -s simbongsa/hey_doopal_msg                        hey_doopal_msg
ln -s simbongsa/assistive_detection                   assistive_detection
ln -s simbongsa/robot_control                         robot_control
ln -s simbongsa/vla/od_msg                            od_msg
ln -s simbongsa/vla/voice_processing/voice_processing voice_processing
```

Python 의존성을 설치합니다.

```bash
pip install mediapipe ultralytics
pip install pyaudio openai langchain langchain-openai python-dotenv gtts openwakeword scipy numpy
pip install pymodbus
pip install -r simbongsa/ui_db/requirements.txt
```

ROS 의존성을 설치합니다.

```bash
cd ~/ws_cobot_pjt/ws_dsr
rosdep install --from-paths src --ignore-src -r -y
```

> `dsr_msgs2`와 두산 ROS 2 드라이버(`DR_init`, `DSR_ROBOT2`)가 같은 워크스페이스에 있거나
> 이미 빌드·source 된 상태여야 합니다. (이 저장소에는 미포함)

## 빌드

인터페이스 패키지를 먼저 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ws_cobot_pjt/ws_dsr
colcon build --symlink-install --packages-select hey_doopal_msg od_msg
source install/setup.bash
```

전체 패키지를 빌드합니다.

```bash
colcon build --symlink-install
source install/setup.bash
```

## 실행 방법

### 1. 인식 노드

RealSense와 두산 로봇 bringup이 먼저 실행되어 있어야 합니다.

```bash
ros2 launch assistive_detection detection.launch.py
```

개별 실행:

```bash
ros2 run assistive_detection hand_detection
ros2 run assistive_detection hand_tracking
ros2 run assistive_detection object_detection
```

### 2. 음성 처리 노드

`resource/.env`에 OpenAI API 키를 넣습니다. (저장소에는 미포함)

```bash
echo "OPENAI_API_KEY=sk-..." > vla/voice_processing/voice_processing/resource/.env
```

```bash
ros2 run voice_processing get_keyword
ros2 run voice_processing tts
```

### 3. 로봇 제어 노드

```bash
python3 -m robot_control.robot_control
```

### 4. 웹 대시보드 + Redis Bridge

```bash
cd ~/ws_cobot_pjt/ws_dsr/src/simbongsa/ui_db
cp .env.example .env      # 값 채우기 ("설정" 참고)
./start_ui_db.sh          # Redis 컨테이너 + Flask + Bridge 동시 실행
./logs_ui_db.sh
./stop_ui_db.sh
```

수동 실행:

```bash
docker compose up -d redis
python3 app.py                       # http://<LAN-IP>:5000
ros2 run <설치위치> ros_object_bridge
```

## 기본 실행 흐름

1. `ros_object_bridge` 실행 (Redis 준비)
2. `object_detection`, `hand_detection`, `hand_tracking` 실행
3. `robot_control` 실행
4. `get_keyword`, `tts` 실행
5. "Hey Doopal" 호출 후 명령 발화 → `get_keyword`가 `/get_keyword`(VoiceKeyword) 호출
6. `robot_control`이 `/yolo_scan_request`로 책상 스캔 → 결과가 Redis에 저장
7. `robot_control`이 대상 pose를 조회(`/get_fixed_pose`, `/get_scan_case`)하고 이동
8. `/grip_bounding_box` 또는 `/find_target_order`로 물체 재확인 후 그립
9. `/find_hand_order`로 손 위치 탐색 → `/arrived_goal` 호출로 `hand_tracking` 시작
10. SpeedL 손 추종으로 접근, `/task_completed` 발행, 물체 전달

## 노드 설명

### `object_detection` (`object_detection_node`)

책상 스캔과 특정 물체 탐색을 담당하는 통합 비전 노드입니다.
카메라 구독·좌표 변환 로직을 공용으로 두고, 서비스 2개와 액션 1개를 얹었습니다.

주요 역할:

* RGB/Depth 동기 구독(`ApproximateTimeSynchronizer`), `CameraInfo` 구독
* YOLO 세그멘테이션 추론(`my_seg_best.pt`)
* `CameraToBaseTransformer`로 카메라 좌표 → base_link mm 좌표 변환 (TF `base_link → link_6`)
* `yolo_scan_request` 서비스: 웨이포인트마다 1프레임 스캔, `/assistive/object_detection` 발행 후 응답
* `grip_bounding_box` 서비스: 그립 직전 재확인, 좌표·bbox·raw depth·grip 각도 반환
* `find_target_order` 액션: 대상 물체를 찾을 때까지 최신 프레임을 반복 확인, `state` feedback

### `hand_detection` (`mediapipe_palm_3d_action_server`)

MediaPipe로 손바닥을 검출해 3D 좌표를 반환하는 액션 서버입니다.

주요 역할:

* `/find_hand_order`(FindOrder) 액션 서버, goal `target_name = "hand"`
* 손바닥 중심 픽셀 + Depth로 카메라 좌표 계산, NPY 행렬로 base_link mm 좌표 변환
* `smoothing_alpha`, `stable_frames`로 좌표 안정화 후 소수 둘째 자리까지 반환
* SpeedL 안전 연동 신호 발행: `/find_hand_order/active`, `/find_hand_order/succeeded` (Bool)
* 최종 base Z는 `max(0, z - 250mm)`로 보정

### `hand_tracking` (`realsense_hand_image_servo_follow_service`)

이미지 서보 방식으로 손을 추종하는 SpeedL 서비스 노드입니다.
절대 손 좌표와 TCP 좌표를 빼는 방식이 아니라 영상 오차 기반으로 속도를 생성합니다.

주요 역할:

* `/arrived_goal`(Trigger) 서비스: 호출되면 손 추종 세션 시작
* 손바닥 중심을 화면 가로 중앙·세로 2/3 지점에 맞추도록 SpeedL 속도 생성
* 목표 픽셀 근처일 때만 Depth 감소, `target_depth_mm` 이하가 되면 정지
* NPY의 회전행렬만 사용해 `v_base = R_base_link6 @ R_link6_camera @ v_camera`
* `SpeedlStream`을 `/dsr01/speedl_stream`으로 발행, `/hand_tracking_request` · `/hand_arrived` 발행
* 손 스캔 중(`/find_hand_order/active`)에는 SpeedL을 0으로 강제 정지

### `robot_control` (`robot_control_node`)

음성 명령을 받아 전체 작업을 순서대로 수행하는 상태 머신 메인 노드입니다.

주요 역할:

* `/get_keyword`(VoiceKeyword) 서비스로 음성 명령 수신, `/ungrip`(Trigger) 서비스 제공
* 클라이언트: `/yolo_scan_request`, `/grip_bounding_box`, `/arrived_goal`, `/get_fixed_pose`, `/get_scan_case`
* 액션 클라이언트: `/find_target_order`, `/find_hand_order` (FindOrder)
* 진행 상태 발행: `/table_scan_finished`, `/hand_scan_start`, `/hand_scan_finished`,
  `/task_completed`, `/table_rescan_started`, `/table_rescan_finished`, `/say`, `/robot_error_status`
* Modbus TCP 그리퍼 제어 (`192.168.1.1:502`)
* 로봇 `dsr01` / `m0609`, 속도 500 / 가속도 60
* `cone_scan.ConeScanner`로 대상 주변을 원뿔형으로 틸트 스캔하며 `FindOrder` 반복 호출

### `get_keyword` (`get_keyword_node`)

웨이크워드 감지부터 의도 분석까지 담당하는 음성 입력 노드입니다.

주요 역할:

* openWakeWord로 "Hey Doopal" 감지 (threshold 0.5)
* STT(`stt.STT`, Whisper `whisper-1`)로 발화 텍스트화
* LangChain `ChatOpenAI` + `PromptTemplate`으로 대상/동작 의도 분석
* `/get_keyword`(VoiceKeyword) 클라이언트로 로봇 제어에 명령 전달, `/ungrip` 클라이언트
* `/say`(TTS), `/ui_chat_log`(UI 대화 로그) 발행
* 로봇 상태 토픽(`/robot_status`, `/table_scan_finished`, `/hand_*` 등) 구독으로 대화 타이밍 제어
* `OPENAI_API_KEY`는 `resource/.env`에서 로드

### `tts` (`tts_node`)

음성 피드백을 재생하는 출력 노드입니다.

주요 역할:

* `/say`(std_msgs/String) 구독, 큐 + 워커 스레드로 순차 재생
* gTTS로 합성 후 `ffplay`로 재생
* 특수 토큰 처리: `STOP_SOUND`(정지), `BEEP::`(beep.mp3), `SCAN::`(스캔 효과음 반복)
* 리소스 경로는 `voice_processing` 패키지 share의 `resource/`

### `ros_object_bridge` (`assistive_robot_redis_bridge`)

ROS 2와 Redis/Flask UI 사이를 잇는 Bridge 노드입니다.

주요 역할:

* 구독: `/assistive/object_detection`, `/assistive/object_moved`, `/assistive/vla_state`,
  `/ui_chat_log`, `/hand_tracking_request`, `/hand_arrived`, `/task_completed`,
  `/assistive/system_log`, `/rosout`(키워드 필터)
* 인식 결과·상태·대화를 Redis에 저장
* 서비스 제공: `/assistive/get_db_data`(GetDbData), `/assistive/get_object_pose`(GetObjectPose),
  `/get_fixed_pose`(GetFixedPose), `/get_scan_case`(GetScanCase)
* 하위 호환 별칭: `/assistive/get_fixed_pose`, `/assistive/get_scan_case`
* `/rosout` 로그 중 `ROSOUT_LOG_FILTER` 키워드에 맞는 항목만 관리자 런타임 로그로 기록

### `cone_scan.ConeScanner`

독립 노드가 아니라 `robot_control`이 사용하는 헬퍼 클래스입니다.
대상 pose를 중심으로 `scan_tilt_angle`(기본 20°)만큼 기울여 `scan_point_count`(기본 8) 지점을
`amovel`로 이동하며 각 지점에서 `FindOrder` 액션을 호출해 물체를 탐색합니다.

## 인터페이스

### Message

#### `TargetPoint.msg`

```text
float64 x
float64 y
float64 z
```

base_link 기준 목표 좌표(mm)를 표현합니다.

### Service

#### `VoiceKeyword.srv`

```text
string target
string goal
---
bool accepted
```

음성 명령(대상 물체와 동작)을 로봇 제어 노드로 전달합니다.

#### `ScanRequest.srv`

```text
string waypoint_id
---
bool success
string message
int32 detected_count
```

한 웨이포인트에서 1프레임 스캔을 수행하고 저장된 객체 수를 반환합니다.
물체가 0개여도 카메라가 정상이면 `success = true`입니다.

#### `GripBoundingBox.srv`

```text
string target
---
float64[3] coordinate       # 로봇 베이스 좌표계 [x, y, z], 이동용
float32 bbox_width          # 픽셀 너비
float32 bbox_height         # 픽셀 높이
float32 camera_depth_z      # 카메라 raw depth, 힘 계산용
float32 grip_angle_deg      # 그립 각도
bool is_find                # 대상 발견 여부
```

그립 직전에 대상이 실제로 있는지 재확인하고 정밀 정보를 반환합니다.

#### `GetDbData.srv`

```text
string data_type   # object, objects, fixed_point(s), scan_case(s), conversations
string name        # 단건 조회 이름 (conversations는 조회 개수)
---
bool success
string json_data
string message
```

Redis에 저장된 데이터를 종류별로 조회하는 범용 서비스입니다.

#### `GetObjectPose.srv`

```text
string object_name
---
bool success
bool has_pose
float64[6] pose            # [obj_x, obj_y, obj_z (mm), rx, ry, rz (deg)]
string coordinate_unit
string angle_unit
string frame_id
string json_data
string message
```

#### `GetFixedPose.srv`

```text
string pose_name           # pose1, pose2, pose3, hand_scan 등
---
bool success
float64[6] pose            # Doosan posx [x_mm, y_mm, z_mm, rx, ry, rz (deg)]
string coordinate_unit
string angle_unit
string frame_id
string json_data
string message
```

#### `GetScanCase.srv`

```text
string case_name           # CASE_1 등
---
bool success
string pose_name_1
float64[6] pose_1
string pose_name_2
float64[6] pose_2
string pose_name_3
float64[6] pose_3
string coordinate_unit
string angle_unit
string frame_id
string json_data
string message
```

DB waypoints 배열의 앞 세 pose를 반환합니다.

#### `od_msg/SrvDepthPosition.srv`

```text
string target
---
float64[] depth_position
```

### Action

#### `FindOrder.action`

```text
string target_name
---
bool found
float64[3] coordinate      # [x_mm, y_mm, z_mm]
string message
---
string state               # searching / detected / stabilizing ...
```

물체 탐색(`find_target_order`)과 손 탐색(`find_hand_order`)에 모두 사용합니다.

#### `OptimizeGripPose.action`

```text
string target
---
bool success
string message
---
bool is_find
float64 bbox_width
float64 bbox_height
float64 camera_depth_z
```

Object Detection이 프레임마다 갱신한 bbox/depth를 feedback으로 전달합니다.

## 개발 환경

권장 환경:

* Ubuntu 22.04
* ROS 2 Humble
* Python 3.10
* OpenCV / NumPy / SciPy
* MediaPipe / Ultralytics(YOLO)
* PyAudio / openWakeWord / OpenAI / LangChain / gTTS
* pymodbus (그리퍼)
* Flask / redis / python-dotenv (ui_db)
* Doosan ROS 2 패키지

필요 ROS 2 패키지:

* `rclpy`, `std_msgs`, `std_srvs`, `sensor_msgs`, `geometry_msgs`, `action_msgs`
* `cv_bridge`, `message_filters`, `tf2_ros`, `ament_index_python`
* `launch`, `launch_ros`
* `dsr_msgs2` 및 두산 드라이버(`DR_init`, `DSR_ROBOT2`)
* `hey_doopal_msg`, `od_msg`

외부 서비스:

* OpenAI API (`OPENAI_API_KEY`) — `voice_processing`의 STT와 의도 분석에 필요

## 설정

### `assistive_detection/config/hand_nodes.yaml`

| 항목 | 설명 |
|---|---|
| `base_frame` | 좌표 기준 프레임 (기본 `base_link`) |
| `calibration_frame` | Hand-Eye 기준 프레임 (기본 `link_6`) |
| `smoothing_alpha` / `stable_frames` | 손 좌표 스무딩 및 안정화 프레임 수 |
| `position_filter_alpha` | 추종 시 위치 필터 계수 |
| `target_depth_mm` | 손 앞에서 정지할 목표 Depth (기본 230.0) |
| `max_forward_speed_mm_s` / `max_total_speed_mm_s` | SpeedL 속도 한계 (기본 500.0) |

### `robot_control/robot_control/robot_control.py` 상수

| 항목 | 값 | 설명 |
|---|---|---|
| `GRIPPER_IP` / `GRIPPER_PORT` | `192.168.1.1` / `502` | Modbus TCP 그리퍼 |
| `ROBOT_ID` / `ROBOT_MODEL` | `dsr01` / `m0609` | 두산 로봇 |
| `VELOCITY` / `ACCELERATION` | `500` / `60` | 이동 속도·가속도 |

### `ui_db/.env`

`ui_db/.env.example`을 복사해 사용합니다.

| 키 | 설명 |
|---|---|
| `FLASK_SECRET_KEY` | Flask 세션 서명 키 (긴 랜덤 문자열) |
| `ADMIN_USERNAME` / `ADMIN_PASSWORD` | 관리자 로그인 |
| `REDIS_HOST` / `PORT` / `DB` / `USERNAME` / `PASSWORD` / `SSL` | Redis 접속. `docker-compose.yml`이 `REDIS_PASSWORD`로 `requirepass` 설정 |
| `SEED_DEMO_OBJECTS` | 데모 객체 시드 여부 |
| `RUNTIME_LOG_FILE` / `MAX_BYTES` / `BACKUP_COUNT` | 관리자 런타임 로그 파일 회전 |
| `ROSOUT_LOG_FILTER` | `/rosout`에서 관리자 로그로 남길 노드명/키워드 |

> `.env`는 커밋하지 마세요. `.gitignore`가 `.env`, `.venv/`, `__pycache__/`, 런타임 로그를 제외합니다.

## 동봉 데이터 파일

| 파일 | 설명 |
|---|---|
| `assistive_detection/assistive_detection/my_seg_best.pt` | 학습된 YOLO 세그멘테이션 모델 (약 20 MB) |
| `assistive_detection/assistive_detection/T_gripper2camera.npy` | Hand-Eye 캘리브레이션 행렬 (link_6 기준) |
| `vla/voice_processing/voice_processing/resource/hey_doopal_final.tflite` | openWakeWord 웨이크워드 모델 |
| `vla/voice_processing/voice_processing/resource/beep.mp3`, `drum.mp3` | TTS 효과음 |

두 인식용 파일은 파이썬 패키지 내부에 설치되며 각 노드가 `Path(__file__)` 기준으로 자동 탐색합니다.

## 주의 사항

* `robot_control/setup.py`의 `console_scripts`가 실제 파일과 다릅니다
  (`final_robot_control`, `test2_robot_control` → 존재하지 않는 모듈).
  현재는 `python3 -m robot_control.robot_control`로 실행하거나 엔트리포인트를
  `robot_control.robot_control:main`으로 수정해야 합니다.
* `voice_processing/setup.py`의 `control_server`, `ui_test` 엔트리포인트도
  저장소에 없는 모듈을 가리킵니다. 동작하는 것은 `get_keyword`, `tts`입니다.
* `robot_control`, `od_msg`, `voice_processing`의 `package.xml` 라이선스/설명이
  템플릿 기본값(`TODO`)입니다.
* 실제 로봇 연결 전에 다음을 확인하세요.
  * 카메라 좌표계와 로봇 좌표계 보정 (`T_gripper2camera.npy`, TF `base_link → link_6`)
  * 그리퍼 TCP 오프셋 (`object_detection` / `hand_detection`의 250 mm 보정값)
  * SpeedL 속도·가속도 한계, emergency stop 동작
  * 마이크 `device_index` (`voice_processing/MicController.py`, 기본 4)
  * 생성된 좌표가 작업 영역 안에 있는지 여부
* 이 저장소는 원본 `sonnanlo2125-a11y/simbongsa`의 fork를 독립 저장소로 전환한 것으로,
  colcon 워크스페이스가 아니라 패키지 소스 모음입니다.

## 팀

두산로보틱스 로키 · 협동 프로젝트 2 · **D-2조** — 김현우, 박현정, 서강민, 장동일
