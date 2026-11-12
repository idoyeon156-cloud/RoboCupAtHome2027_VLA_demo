# 2차 MoveIt 다면체 충돌 검사

기본 실행은 양팔 전체에 **구간별 볼록 다면체 메시**를 사용합니다. 팔 하나당 31개, 양팔 62개이며 베이스·팔·손목·그리퍼 몸체·양쪽 손가락에 모두 적용합니다. 원본 visual/STL과 관절, 베이스 간격은 유지합니다.

원본 삼각형을 링크 로컬 구간에서 자른 뒤 각 구간을 감싸는 볼록 다면체를 만듭니다. 큰 사선 면의 방향을 유지하므로 박스가 채우던 빈 모서리를 줄입니다. 복잡한 링크는 조각을 나눈 상태로 유지합니다. 하나의 큰 볼록 껍질로 합치지 않습니다. 다만 각 조각 내부의 작은 오목한 홈까지 정확히 재현하는 원본 메시 검사는 아닙니다.

지지 평면을 바깥으로 5mm 이동해 여유를 메시 파일에 포함했습니다. 원본 면은 각 조각의 경계에서 최소 약 5mm 안쪽에 들어갑니다. 다면체 모서리 방향의 여유는 5mm보다 클 수 있습니다. 화면의 색칠된 삼각형과 충돌 검사에 쓰는 메시가 같습니다. 보이지 않는 추가 5mm는 없습니다. 5mm는 실측 제동거리로 검증된 값이 아닙니다.

다면체는 MoveIt의 Bullet 검사기로 각 조각을 독립적인 볼록 입체로 검사합니다. 다른 입체 안에 완전히 들어간 경우도 검사합니다. 이동 구간 검사는 여전히 기존의 유한 간격 표본 검사이며 연속 충돌 검사를 추가한 것은 아닙니다.

## 기본 실행 — 시뮬과 실물에 같은 다면체 적용

```bash
bash run_moveit_visual_demo.sh
bash run_moveit_real_guard.sh
```

시뮬은 관절 조절기와 RViz가 열립니다. 충돌 메시를 파랑/주황 면과 가는 삼각형 선으로 표시하고, 충돌한 링크의 전체 조각을 빨갛게 표시합니다. 빨간 구는 접촉 위치입니다. `Geometry overlap`은 실제 외함이 아닌 검사 형상의 겹침 깊이입니다.

실물용은 기본 감시 모드이며 드라이버를 실행하지 않습니다. 설정 변경 후 재시작해야 합니다. 형상·정지 동작의 실물 검증은 별도입니다.

기본 다면체 모드에는 박스 편집 창이 열리지 않습니다. 기존 `collision_tuning.json`은 기본 다면체 모드에서 자동으로 읽지 않습니다. 명시적으로 전달하면 박스 모드 전용 설정이라는 오류를 표시합니다. 다면체는 생성된 STL을 사용하므로 박스 크기 조절로 변경하지 않습니다.

## 다면체 재생성

```bash
python3 tools/generate_collision_meshes.py --margin 0.005
```

생성할 때만 NumPy와 SciPy가 필요합니다. 실행용 ZIP에는 생성된 STL이 들어 있어 다시 생성할 필요가 없습니다. `meshes/collision/`은 실제 검사 메시, `config/dual_piper_meshes.urdf`는 조각의 링크 연결, `config/collision_meshes.json`은 생성 조건과 지지 평면입니다. 재생성 뒤 다시 빌드/실행하세요.

## 이전 박스 모드와 수동 편집

```bash
bash run_moveit_visual_demo.sh geometry:=boxes editor:=true
bash run_moveit_real_guard.sh geometry:=boxes
# 기존 박스 설정을 명시하는 경우
bash run_moveit_real_guard.sh geometry:=boxes collision_config:=/절대/경로/collision_tuning.json
```

아래의 박스 수동 편집·JSON 설명은 `geometry:=boxes`를 선택했을 때만 적용합니다. 박스 모드는 기존 MoveIt/FCL 검사를 유지합니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/Robocup-Home-2027_VLA_Tutorial/collision
source .venv/bin/activate
# MoveIt이 없다면 설치:
# sudo apt install ros-humble-moveit
python /usr/bin/colcon build --symlink-install --packages-select piper_dual_description piper_collision_guard piper_moveit_guard
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

`.venv`/Pinocchio 설치는 기존 piper_collision_guard 문서를 참고합니다.

## 마우스로 충돌 자세 확인 (실제 로봇 연결 없음)

```bash
ros2 launch piper_moveit_guard moveit_box_demo.launch.py geometry:=boxes editor:=true
```

왼팔·오른팔이 두 열로 나란히 배치된 관절 조절기와 RViz가 열립니다. 각 팔의 joint1~8을 한 화면에서 조절하며 회전은 도(°), 손가락은 mm로 입력합니다. 로봇 외형은 불투명하게 표시됩니다. 충돌 박스는 옅은 채움과 선명한 테두리로 표시됩니다(왼팔 파란색, 오른팔 주황색). 위쪽 상태 글씨가 초록색 CLEAR / 빨간색 BLOCKED로 바뀌고 충돌한 링크 쌍을 표시합니다. 빨간 점은 접촉 위치이며 `Box overlap`은 보고된 접촉점의 박스 겹침 깊이(mm)입니다. 이 값은 실제 외함의 침범 깊이가 아닙니다.
이 데모는 요청 자세를 그대로 보여주는 **MoveIt 검사 전용 데모**이며, 슬라이더 이동을 막지 않습니다.
기존 Pinocchio의 허용/요청 자세 데모와는 별개의 실행 구성입니다.

다른 터미널에서 같은 작업공간을 활성화한 후:

```bash
ros2 topic echo /moveit_guard/status
```

`pose_collision: true`, `reason: box_collision`이면 박스가 겹칩니다. `pair`는 충돌한 좌우 링크이며 `penetration_depth_m`은 해당 접촉점의 박스 겹침 깊이(m)입니다.
손가락은 joint7/joint8을 각각 조절합니다. 실제 대칭 개폐는 joint8=-joint7입니다.
GUI 없이 검사 노드만 확인하려면 `gui:=false rviz:=false`를 추가합니다.

## 링크별 충돌 박스 수동 튜닝

`geometry:=boxes editor:=true`로 실행하면 **박스 편집 창**이 열립니다. 관절 조절기로 자세를 바꾸면서 별도 편집 창에서 다음을 조정합니다.

1. `left_link2`처럼 링크와 박스 번호를 선택합니다. 각 링크의 박스는 분할 구간 순서대로 표시됩니다. `link2`, `link3`, `gripper_base`는 각각 5개, `link4`는 3개, `link5`는 2개, 손가락 `link7`, `link8`은 각각 4개입니다. 선택 링크는 노란색, 충돌한 링크는 빨간색으로 표시됩니다.
2. **박스 본체 크기 XYZ(mm)**, **중심 위치 XYZ(mm)**, **회전 R/P/Y(°)**, **각 면 추가 여유(mm)**를 입력합니다. 중심/회전은 해당 링크 좌표계 기준입니다.
3. 값이 바뀌면 시뮬의 MoveIt 모델과 RViz 박스가 함께 갱신됩니다. 편집 창의 **MoveIt 반영 완료**를 확인합니다.
4. 복잡한 부위는 **박스 추가**로 작은 박스를 여러 개 배치합니다(링크당 최대 8개). 추가한 박스는 기존 박스를 복사하므로 크기·위치를 각각 조절하세요. 링크당 박스 1개는 유지합니다.
5. **반대 팔 같은 링크에 복사**는 선택한 링크의 모든 박스를 반대쪽에 복사합니다. **이 링크 기본값**으로 원복할 수 있습니다.
6. **저장**하면 실행 폴더에 `collision_tuning.json`을 만듭니다. 다른 경로는 **다른 이름으로 저장**을 사용합니다.

충돌 검사 전체 크기는 `박스 본체 크기 + 2×면당 여유`입니다. 기본 여유 5mm이면 각 축 전체 길이에 10mm가 추가됩니다. 편집 창에 최종 크기도 표시합니다.

**기존 `collision_tuning.json`은 새 기본값보다 우선합니다.** 예전의 큰 박스 설정을 사용 중이면 편집기에서 양팔 `link2`, `link3`, `link4`, `link5`, `gripper_base`, `link7`, `link8`을 각각 선택하고 **이 링크 기본값**을 누른 뒤 저장하세요. 한쪽씩 복원하고 **반대 팔 같은 링크에 복사**해도 됩니다. 다른 링크의 수동 설정은 그대로 유지할 수 있습니다. 전체를 새 기본값으로 바꾸려면 기존 파일을 다른 이름으로 백업한 뒤 실행하세요.

저장한 설정은 같은 폴더에서 데모/실물용 스크립트를 재실행하면 자동으로 읽습니다. 다른 ZIP 폴더나 PC에서 사용할 때는 `collision_tuning.json`도 복사하세요. 경로를 직접 지정할 수도 있습니다.

```bash
bash run_moveit_visual_demo.sh geometry:=boxes editor:=true collision_config:=/절대/경로/collision_tuning.json
bash run_moveit_real_guard.sh geometry:=boxes collision_config:=/절대/경로/collision_tuning.json
```

실물용은 **시작할 때만** 설정을 읽습니다. 편집 창의 실시간 변경은 데모 전용이며 실행 중인 실물용 노드에는 적용되지 않습니다. 설정을 바꿨다면 실물용을 종료하고 재시작해야 합니다. 이 편집기는 MoveIt 박스만 변경하며 **1차 Pinocchio 보호 구·거리 임계값은 변경하지 않습니다.**

편집 창을 숨기려면 데모에 `editor:=false`를 추가합니다. 박스를 줄이거나 이동하면 기존 외형 중 검사에서 빠지는 부분이 생길 수 있으므로 실제 외형과 대조해 조정하세요. 좌우 링크 쌍 전체를 검사에서 제외하는 기능은 넣지 않았습니다.

## 기존 1차 장치와 함께 감시

```bash
ros2 launch piper_moveit_guard two_stage_guard.launch.py
```

기본 monitor 모드에서는 두 노드 모두 Piper 명령 publisher를 만들지 않습니다.
실제 피드백 `/joint_states_left`, `/joint_states_right`, 목표 `/teleop/left_target`, `/teleop/right_target`을 구독합니다.
두 노드의 상태를 각각 `/collision_guard/status`, `/moveit_guard/status`에서 확인합니다.

형상/정지 여유 및 드라이버 정지 동작을 검증한 뒤 명시적으로 전달을 켜는 구성:

```bash
ros2 launch piper_moveit_guard two_stage_guard.launch.py enforce:=true geometry_confirmed:=true
```

명령 경로:

```text
/teleop/*_target
  → Pinocchio 거리 검사
  → /pinocchio_checked/*_target
  → MoveIt 박스 검사
  → /joint_ctrl_cmd_left, /joint_ctrl_cmd_right
  → Piper 드라이버
```

이 launch는 실제 드라이버를 실행하지 않습니다. 텔레옵/SDK 직접 명령이나 다른 publisher가 최종 토픽을 우회하지 않도록 연결해야 합니다.
기존 `guard.launch.py`를 함께 실행하지 마세요. two_stage_guard가 1차 노드도 실행합니다.

입력은 6개 관절(rad)+`gripper` 전체 개구 폭(m, 0~0.07), 유효한 ROS header timestamp가 필요합니다.
MoveIt 손가락은 +폭/2, -폭/2로 변환합니다. 실제 드라이버 변환은 실물과 대조해야 합니다.
양팔 소스 시간차 50ms, 입력 유효기간 150ms를 초과하거나 잘못된 입력/검사 시간 초과/충돌이 발생하면 차단합니다.
양팔 각자의 현재→목표 진행률 조합을 격자로 검사하여 한쪽 팔이 지연되는 경우도 표본 검사합니다.
기본 간격은 관절 0.05rad, 손가락 2mm, 최대 2500개 조합, 계산 예산 50ms입니다.
예산 초과는 안전 판정이 아니라 차단입니다. 50Hz는 타이머 설정이며 실제 처리율은 계산량에 따라 달라집니다.

충돌/오류 래치는 자동 해제되지 않습니다. 안전한 실제 자세에서 다음 순서로 해제하고 새 목표를 보냅니다:

```bash
ros2 service call /collision_guard/reset std_srvs/srv/Trigger '{}'
ros2 service call /moveit_guard/reset std_srvs/srv/Trigger '{}'
```

차단 시 최신 유효 피드백의 위치 유지 목표를 반복 요청합니다. 피드백이 오래되면 명령을 발행하지 않습니다.
**위치 유지와 stop_requested는 하드웨어 비상정지가 아닙니다.** 프로세스 종료 뒤 기존 명령이 남을 수 있으므로 드라이버의 독립 watchdog과 검증한 정지 기능이 필요합니다.

## 박스 크기 변경

`config/collision_boxes.csv`: 각 박스의 소속 링크/중심/전체 크기/면당 여유(m). 같은 링크 이름이 여러 행에 나타납니다.
`config/dual_piper_boxes.urdf`: MoveIt에 실제로 로드하는 모델.

```bash
python3 tools/generate_collision_boxes.py --margin 0.005
```

원본 STL에서 구간별 충돌 박스와 SRDF를 재생성합니다. `tools/generate_collision_boxes.py`의 `SECTIONS`는 링크 로컬 좌표계에서의 분할 축/경계(m)입니다. 링크당 최대 8개의 박스를 지원하며, 기본 `link2`, `link3`, `gripper_base`는 5개, `link4`는 3개, `link5`는 2개, `link7`, `link8`은 4개입니다. 이 명령은 박스 수동 수정 내용을 덮어씁니다.
수정 후 다시 빌드하고 노드를 재시작하세요. RViz에서 외형을 완전히 덮는지 먼저 확인합니다.

## 범위와 한계

- ACM은 같은 팔 내부 쌍을 제외합니다. 좌우 100개 링크 쌍은 모두 검사합니다.
- 이 버전은 양팔 상호 충돌 전용입니다. 같은 팔 내부, 환경/사람, 파지물은 검사하지 않습니다.
- 긴 링크·손목·그리퍼는 여러 박스로 나눴지만 곡면 주변에는 여전히 빈 공간이 포함될 수 있습니다. 필요하면 편집기로 각 구간의 형상을 더 조정합니다.
- 유한 간격의 격자 검사는 연속 충돌 보증이 아닙니다. 실제 제어기의 이동 궤적이 관절 선형 보간에서 벗어나는 경우는 별도 검증이 필요합니다.
- 5mm 여유는 실측 제동거리에서 정한 값이 아닙니다. 속도/지연/외형 오차에 맞춰 검증해야 합니다.
- `/moveit_guard/check_state_validity`는 테스트/도구용 서비스입니다. 좌우 joint1~8 총 16개를 모두 넣어야 하며 양팔 충돌과 관절 범위만 검사합니다.

공식 API 참고: https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html

## 개발 검증

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper_moveit_guard/test/test_geometry.py -q
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 RUN_MOVEIT_GUARD_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper_moveit_guard/test -q
```

ROS 테스트는 실제 드라이버를 실행하지 않고 별도 도메인에서 가짜 관절 상태만 사용합니다.

## 간편 실행 스크립트

저장소 또는 ZIP을 푼 폴더에서 실행합니다. 필요한 ROS 의존성과 가상환경은 먼저 설치해야 하며, 스크립트가 소스를 빌드한 뒤 실행합니다.

```bash
# 시뮬/RViz 확인용: 실제 로봇 명령 없음
bash run_moveit_visual_demo.sh

# 실물 연결용: 기본 감시 모드, 드라이버는 별도 실행
bash run_moveit_real_guard.sh
```

실물용은 실물 검증 전 프로토타입입니다. 명령 전달은 검증 후에만 `enforce:=true geometry_confirmed:=true`를 추가합니다.
