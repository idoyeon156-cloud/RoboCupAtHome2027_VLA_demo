# 충돌 제한 시각적으로 보는 데모 — ROS2 Humble / RViz2

이 실행 파일은 **충돌 제한 시각화 데모**입니다. 실제 Piper SDK/CAN/드라이버를 실행하지 않으며, 하드웨어 명령 토픽에 발행하지 않습니다. Pinocchio로 계산한 보호 구 간격에 따라 마우스로 요청한 자세가 허용되는지 확인합니다. 물리·제동 시뮬레이터 또는 MoveIt 메시 충돌 검사 데모는 아닙니다.

## 화면에서 보이는 것

- **관절 조절 창:** `left_joint1`~`left_joint6`, `right_joint1`~`right_joint6` 슬라이더를 마우스로 움직입니다. 숫자로 각도(rad)를 입력할 수도 있습니다.
- **불투명 회색 로봇:** 현재 허용된 자세. 위험한 요청에는 마지막 허용 자세에 머뭅니다.
- **반투명 로봇:** 슬라이더로 요청한 자세. 제한을 넘는 자세도 보이므로 어디서 걸렸는지 비교할 수 있습니다.
- **반투명 보호 구:** 요청 자세의 보호 구. 통과 시 초록색, 제한 시 빨간색입니다.
- **선:** 요청 자세에서 최소 여유를 만드는 두 구의 표면 연결선.
- **상태 글자:** `CLEAR` / `LIMIT REACHED`, 요청 자세의 표면 간격, 이동 구간 최소 간격, 정지 여유(mm).

이 데모는 팔 끝을 직접 잡아 끄는 IK 핸들이 아니라 **관절 슬라이더 방식**입니다. 손가락 슬라이더도 원본 URDF에 따라 보이지만, 1차 보호 구는 손가락 개폐를 자동 추종하는 형상이 아닙니다.

## Ubuntu에서 처음 한 번 준비

터미널에서 실행합니다. 이미 저장소를 받았다면 clone 대신 해당 폴더에서 `git pull`하세요. 다른 ROS 작업공간과 겹치지 않는 새 터미널을 권장합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~
git clone https://github.com/idoyeon156-cloud/Robocup-Home-2027_VLA_Tutorial.git
cd Robocup-Home-2027_VLA_Tutorial
sudo apt update
sudo apt install python3-venv ros-humble-rviz2 ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install pin==2.7.0
python /usr/bin/colcon build --symlink-install --packages-select piper_dual_description piper_collision_guard
```

## 실행

위 빌드에 사용한 가상환경을 활성화하고 작업공간 설정을 읽습니다. ROS domain 178은 실기와 데모를 구분하기 위한 값입니다. 같은 domain에서 실제 드라이버를 실행하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/Robocup-Home-2027_VLA_Tutorial
source .venv/bin/activate
source install/setup.bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 ros2 launch piper_collision_guard collision_limit_visual_demo.launch.py
```

RViz와 슬라이더 창이 열립니다.

**현재 채팅에서 준비한 로컬 사본은 이미 빌드되어 있습니다.** 그 사본에서는 다음만 실행하면 됩니다. 공백 있는 경로에서 colcon이 Python 경로를 잘못 해석하는 문제를 피하려고 `~/piper_vla_env`라는 가상환경 별칭도 준비했습니다.

가장 간단한 방법:

```bash
cd "$HOME/문서/ChatGPT/VLA 공부/Robocup-Home-2027_VLA_Tutorial"
bash run_collision_visual_demo.sh
```

스크립트에 `stop_clearance_m:=0.03`, `block_unsafe:=false` 같은 실행 인자를 뒤에 붙일 수도 있습니다. 아래는 같은 작업을 수동으로 실행하는 방법입니다.

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/문서/ChatGPT/VLA 공부/Robocup-Home-2027_VLA_Tutorial"
source "$HOME/piper_vla_env/bin/activate"
source install/setup.bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 ros2 launch piper_collision_guard collision_limit_visual_demo.launch.py
```

이 로컬 사본을 재빌드할 때는 `"$HOME/piper_vla_env/bin/python" /usr/bin/colcon build --symlink-install --packages-select piper_dual_description piper_collision_guard`를 사용합니다. GitHub에서 ~/Robocup-Home-2027_VLA_Tutorial처럼 공백 없는 경로에 새로 clone한 경우 위 일반 설치 방법대로 진행하면 됩니다.

1. 처음에는 양팔이 같은 0자세에 있습니다. 현재 예시 설정의 최소 중심 거리는 175mm, 보호 구 표면 간격은 95mm입니다.
2. 한 팔의 관절을 조금씩 움직이고 반투명 보호 구가 상대 팔에 접근하는 것을 봅니다.
3. 요청 자세 또는 현재 허용 자세에서 요청 자세로 이동하는 보간 구간의 간격이 20mm 이하이면 빨간색과 `LIMIT REACHED`가 표시됩니다.
4. 불투명 로봇은 마지막 허용 자세에서 멈추며 반투명 요청 로봇은 계속 움직일 수 있습니다.
5. 슬라이더를 다시 안전한 쪽으로 움직이면 데모가 자동으로 따라갑니다. 필요하면 slider 창의 Center로 모든 관절을 0으로 돌립니다. Randomize는 큰 불연속 요청으로 경로 제한에 걸리기 쉬워 첫 확인에는 사용하지 않는 편이 좋습니다.

실제 guard는 정지 후 수동 reset이 필요하지만 이 데모는 자세 탐색을 위해 자동 복귀합니다. 이 차이를 실기 재개 로직으로 해석하지 않습니다.

## 임계값 바꾸기

표면 여유를 30mm로 바꾸려면 종료(Ctrl+C) 후:

```bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 ros2 launch piper_collision_guard collision_limit_visual_demo.launch.py stop_clearance_m:=0.03
```

보호 구의 중심/반지름은 [`config/protection.yaml`](config/protection.yaml)을 수정하고 재시작합니다. symlink-install로 빌드했다면 설정 내용 수정에 재빌드가 필요하지 않습니다. 기본 반지름 40mm, 여유 20mm는 시뮬 시작용 예시이며 실측·제동 검증값이 아닙니다.

기본 베이스 원점 간격은 **175mm**로, 원본 STL 폭 80mm를 기준으로 외곽 간격 **95mm**를 반영합니다. 간격만 비교하려면 `base_separation_m:=0.18`처럼 실행 인자를 줄 수 있습니다. 이는 모델의 원점 간격이며, 양쪽 RSP 모델과 Pinocchio에 동일하게 적용됩니다. 원본 URDF 파일은 수정하지 않습니다.

**위험 자세까지 불투명 로봇도 그대로 따라가게 하고 표시만 보고 싶다면:**

```bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 ros2 launch piper_collision_guard collision_limit_visual_demo.launch.py block_unsafe:=false
```

이때는 `INSPECT ONLY`가 표시됩니다. 여전히 실제 로봇 명령은 보내지 않습니다.

## 검증용으로 바로 제한을 만드는 자세

슬라이더 숫자 입력란에 아래 rad 값을 넣으면 양팔 보호 구가 가까워지는 예시를 볼 수 있습니다. 입력 중 다른 경로에서 먼저 제한될 수도 있으며, 이때도 반투명 요청 자세는 표시됩니다.

| 조인트 | 왼팔 | 오른팔 |
|---|---:|---:|
| 1 | 0.24723 | 0.63470 |
| 2 | 0.97564 | 0.72965 |
| 3 | -0.60251 | -0.62731 |
| 4 | 0.19306 | 0.34530 |
| 5 | -0.55266 | -0.00143 |
| 6 | 0.28216 | -0.37092 |

**이 자세를 실제 로봇에 보내지 마세요.** 보호 구 침범 표시를 확인하기 위한 계산 예시입니다.

## 실행이 안 될 때

- `No module named pinocchio`: 빌드 때 쓴 .venv를 활성화했는지 확인합니다. `pip install pinocchio`가 아니라 위의 `pin==2.7.0` 패키지입니다.
- 패키지를 못 찾음: 같은 터미널에서 install/setup.bash를 source합니다.
- 로봇은 안 보이고 구만 보임: RViz Displays의 두 RobotModel을 켜고, Fixed Frame이 world인지 확인합니다. 첫 모델 Description Topic은 `/collision_preview/accepted/robot_description`, 두 번째는 `/collision_preview/ghost/robot_description`, 두 번째 TF Prefix는 `ghost`입니다.
- `qt.qpa.xcb` 등 GUI 오류: Ubuntu 데스크톱 터미널에서 실행합니다. 디스플레이 전달이 없는 SSH 터미널에서는 창이 열리지 않습니다.
- GUI 없이 연결만 점검: 실행 명령 끝에 `gui:=false rviz:=false`를 붙이면 0자세 데이터와 TF만 발행합니다.

## 검증 범위

실제 Pinocchio 모델로 0자세의 간격, 위험 요청 시 허용 자세 동결, 안전 자세 복귀, inspect 모드, 잘못된 관절값 거부를 테스트합니다. ROS launch로 두 모델/관절/TF와 하드웨어 명령 publisher가 없음을 검사합니다. 슬라이더 GUI는 offscreen 모드로 연결을 테스트하며 RViz의 실제 화면 렌더와 실기 모터/물리 제동은 별도 확인이 필요합니다.
