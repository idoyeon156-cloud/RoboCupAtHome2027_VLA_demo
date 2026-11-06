# 2차 MoveIt 직육면체 충돌 검사

ROS2 Humble / MoveIt FCL로 **왼팔과 오른팔 사이**의 부피 충돌을 검사합니다.
각 STL의 링크 좌표계 외곽 범위에서 박스 20개를 만들었고, 각 면에 5mm의 미검증 예시 여유를 추가했습니다.
원본 visual, 관절/관성, 베이스 원점 간격 175mm는 보존합니다.
베이스 실제 메시 간격은 약 95mm, 확장한 충돌 박스 간격은 약 85mm입니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/Robocup-Home-2027_VLA_Tutorial
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
ros2 launch piper_moveit_guard moveit_box_demo.launch.py
```

관절 슬라이더와 RViz가 열립니다. 반투명 박스는 충돌 형상, 원본 외형은 더 옅게 표시됩니다. 위쪽 상태 글씨가 초록색 CLEAR / 빨간색 BLOCKED로 바뀌고 충돌한 링크 쌍을 표시합니다. 빨간 점은 접촉 위치이며 `Box overlap`은 보고된 접촉점의 박스 겹침 깊이(mm)입니다. 이 값은 실제 외함의 침범 깊이가 아닙니다.
이 데모는 요청 자세를 그대로 보여주는 **MoveIt 검사 전용 데모**이며, 슬라이더 이동을 막지 않습니다.
기존 Pinocchio의 허용/요청 자세 데모와는 별개의 실행 구성입니다.

다른 터미널에서 같은 작업공간을 활성화한 후:

```bash
ros2 topic echo /moveit_guard/status
```

`pose_collision: true`, `reason: box_collision`이면 박스가 겹칩니다. `pair`는 충돌한 좌우 링크이며 `penetration_depth_m`은 해당 접촉점의 박스 겹침 깊이(m)입니다.
손가락은 joint7/joint8을 각각 조절합니다. 실제 대칭 개폐는 joint8=-joint7입니다.
GUI 없이 검사 노드만 확인하려면 `gui:=false rviz:=false`를 추가합니다.

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

`config/collision_boxes.csv`: 각 링크 중심/전체 크기/면당 여유(m).
`config/dual_piper_boxes.urdf`: MoveIt에 실제로 로드하는 모델.

```bash
python3 tools/generate_collision_boxes.py --margin 0.005
```

원본 STL에서 충돌 박스와 SRDF를 재생성합니다. 이 명령은 박스 수동 수정 내용을 덮어씁니다.
수정 후 다시 빌드하고 노드를 재시작하세요. RViz에서 외형을 완전히 덮는지 먼저 확인합니다.

## 범위와 한계

- ACM은 같은 팔 내부 쌍을 제외합니다. 좌우 100개 링크 쌍은 모두 검사합니다.
- 이 버전은 양팔 상호 충돌 전용입니다. 같은 팔 내부, 환경/사람, 파지물은 검사하지 않습니다.
- 박스 1개로 감싼 빈 공간 때문에 보수적으로 제한될 수 있습니다. 필요하면 여러 박스로 나눕니다.
- 유한 간격의 격자 검사는 연속 충돌 보증이 아닙니다. 실제 제어기의 이동 궤적이 관절 선형 보간에서 벗어나는 경우는 별도 검증이 필요합니다.
- 5mm 여유는 실측 제동거리에서 정한 값이 아닙니다. 속도/지연/외형 오차에 맞춰 검증해야 합니다.
- `/moveit_guard/check_state_validity`는 테스트/도구용 서비스입니다. 좌우 joint1~8 총 16개를 모두 넣어야 하며 양팔 충돌과 관절 범위만 검사합니다.

공식 API 참고: https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html

## 개발 검증

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper_moveit_guard/test/test_geometry.py -q
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 RUN_MOVEIT_GUARD_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper_moveit_guard/test -q
```

ROS 테스트는 실제 드라이버를 실행하지 않고 별도 도메인에서 가짜 관절 상태만 사용합니다.
