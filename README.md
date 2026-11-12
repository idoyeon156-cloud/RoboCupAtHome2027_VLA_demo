# RoboCup@Home 2027 — Dual PiPER

양팔 PiPER의 URDF, 두 단계 안전장치, 알파 기반 씨리얼 파지·붓기 시뮬레이션과 실물 배포본을 보관합니다.
하드웨어 모델은 **HW**, 실행 소프트웨어는 **SW**에서 찾을 수 있습니다.

```text
HW/
└── URDF/
    ├── original/           # 교체 전 URDF + STL (총 개방량 70 mm)
    └── updated_gripper/    # 교체 후 URDF + STL (새 베이스·플랜지·D405 마운트, 100 mm)
SW/
├── original_gripper/       # 교체 전 모델: 조인트 거리 + MoveIt 메시 안전장치
│   └── legacy/             # 초기 버전 소스와 미리보기 보존
├── updated_gripper/        # 교체 후 모델 + 마운트: 조인트 거리 + MoveIt 메시 안전장치
├── cereal_pour/            # 교체 후 모델 + 마운트: 알파 기반 씨리얼 RViz·실물 배포
└── bundles/                # 각 안전장치의 시뮬·실물용 소스 ZIP
    ├── original_gripper/
    ├── updated_gripper/
    └── cereal_pour/        # 알파 기반 시뮬레이션·실물용 ZIP
```

| 폴더 | 내용 | 안내 |
| --- | --- | --- |
| [HW/URDF/original](HW/URDF/original/) | 교체 전 독립 URDF 패키지 | [모델 빌드](HW/URDF/original/README.md) |
| [HW/URDF/updated_gripper](HW/URDF/updated_gripper/) | 교체 후 그리퍼 + D405 마운트 독립 URDF 패키지 | [모델 빌드](HW/URDF/updated_gripper/README.md) |
| [SW/original_gripper](SW/original_gripper/) | 교체 전 기준의 두 안전장치 | [설치·설정·실행](SW/original_gripper/README.md) |
| [SW/updated_gripper](SW/updated_gripper/) | 교체 후 그리퍼 + 마운트 기준의 두 안전장치 | [설치·설정·실행](SW/updated_gripper/README.md) |
| [SW/cereal_pour](SW/cereal_pour/) | 영점 → 동시 알파 → 파지 → 붓기 → 내려놓기 → 영점, 실물 실행 포함 | [실행](SW/cereal_pour/README.md) |
| [SW/bundles](SW/bundles/) | 대상 기기에서 빌드할 수 있는 소스 배포 ZIP | [ZIP 선택](SW/bundles/README.md) |

## 내려받기와 최초 준비

Ubuntu 22.04 / ROS 2 Humble 환경을 기준으로 합니다. Jetson 등 다른 기기에서도 소스를 내려받은 뒤 **해당 기기에서 빌드**합니다. PC의 가상환경·빌드 산출물은 포함하지 않습니다. Jetson에서의 실행은 별도 검증이 필요합니다.

원하는 위치에서 내려받습니다.

```bash
git clone https://github.com/idoyeon156-cloud/RoboCupAtHome2027_VLA_demo.git
cd RoboCupAtHome2027_VLA_demo
```

ROS 의존성 설치는 최초 한 번 실행합니다.

```bash
sudo apt install ros-humble-moveit ros-humble-rviz2 \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-ros ros-humble-python-qt-binding \
  python3-venv python3-colcon-common-extensions
```

아래 세 작업 공간 중 사용할 폴더로 이동한 뒤 가상환경을 준비합니다. **각 작업 공간마다 최초 한 번** 필요합니다. 실행 스크립트가 그 작업 공간의 ROS 패키지 3개를 자동 빌드합니다.

```bash
# 저장소 루트에서: 교체 후 모델을 사용하는 경우
cd SW/updated_gripper
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
```

교체 전 모델은 `SW/original_gripper`, 씨리얼 재생은 `SW/cereal_pour`에서 같은 준비 절차를 실행합니다. 세 작업 공간은 같은 ROS 패키지 이름을 사용하므로 별도 셸에서 하나씩 실행하세요. 저장소 전체를 하나의 colcon 작업 공간으로 빌드하지 않습니다.

## 실행 명령

아래 `cd`는 **저장소 루트 기준**입니다. 종료는 Ctrl+C입니다.

교체 전 URDF 기준:

```bash
cd SW/original_gripper
bash run_distance_rviz.sh       # 조인트 간 거리 안전장치
# 위 시뮬레이션을 종료한 뒤
bash run_meshes_rviz.sh         # MoveIt 메시 안전장치
```

교체 후 그리퍼 + 마운트 URDF 기준:

```bash
cd SW/updated_gripper
bash run_distance_rviz.sh
# 위 시뮬레이션을 종료한 뒤
bash run_meshes_rviz.sh
```

알파 기반 씨리얼 RViz·실물 배포:

```bash
cd SW/cereal_pour
bash run_meshes_rviz.sh cereal:=true
```

양팔 알파는 `[0,90,0,0,-70,0]°`입니다. 물체 바닥은 베이스 장착면 Z=0, 그릇 파지 높이는 3.5cm이며 위치·크기는 `SW/cereal_pour/config/cereal_task.yaml`에서 수정합니다. 저장 후 재실행하면 IK와 경로 충돌 검사를 다시 수행합니다. [시뮬·실물 ZIP](SW/bundles/README.md#알파-기반-시리얼-파지붓기)을 내려받을 수도 있습니다.

씨리얼 재생은 관절 동작 시각화입니다. 실제 시리얼 입자·유체 시뮬레이션은 포함하지 않습니다.

## 실물용 실행

알파 기반 파지·붓기 실물용:

```bash
cd SW/cereal_pour
bash run_cereal_real.sh
```

기본 monitor이며, 조인트 거리 → MoveIt 메시 가드가 함께 실행됩니다. prepare/start 서비스와 명령 전달 설정은 [CEREAL_TASK.md](SW/cereal_pour/CEREAL_TASK.md)를 참고하세요.


두 안전장치 작업 공간에는 기존 실물 경로도 보존합니다.

```bash
# SW/original_gripper 또는 SW/updated_gripper 안에서
bash run_distance_real.sh       # 1차 거리 검사
# 또는
bash run_meshes_real.sh         # 1차 거리 → 2차 MoveIt 메시 검사
```

기본값은 감시 모드이며 실제 드라이버는 별도로 실행합니다. 명령 차단 활성화, 피드백·목표 토픽, 래치 해제는 각 작업 공간 README에 있습니다.

현재 `updated_gripper`는 신형 그리퍼와 양쪽 D405 마운트 v6를 포함합니다. 별도 마운트 버전 폴더는 만들지 않습니다. [장착 형상과 검증 범위](SW/updated_gripper/SENSOR_MOUNT.md)를 참고하세요.

## 모델과 소스 관리

HW는 모델만 쓰는 경우를 위한 독립 ROS 패키지입니다. SW의 각 작업 공간과 ZIP에는 실행에 필요한 URDF·STL·YAML을 함께 넣어 독립적으로 빌드할 수 있게 했습니다. HW를 수정해도 SW의 모델이 자동 변경되지는 않습니다. SW에서 형상을 바꿀 때는 해당 작업 공간의 원본 모델·메시·`config/robot.yaml`을 수정하고 `python tools/prepare_model.py`로 재생성합니다.

## 이전 경로에서 옮겨진 위치

| 이전 경로 | 새 경로 |
| --- | --- |
| `collision/` | `SW/original_gripper/legacy/` |
| `collision_configurable/` | `SW/original_gripper/` |
| `collision_updated_gripper/` | `SW/updated_gripper/` |
| `cereal_pour_rviz/` | `SW/cereal_pour/` |
| `robot_description/piper_dual_updated_gripper_description/` | `HW/URDF/updated_gripper/` |
| `artifacts/configurable/` | `SW/bundles/original_gripper/` |
| `artifacts/updated_gripper/` | `SW/bundles/updated_gripper/` |
