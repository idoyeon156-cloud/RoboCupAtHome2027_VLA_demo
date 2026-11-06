# 1차 장치: Pinocchio 양팔 거리 감시·명령 제한

**마우스로 자세를 바꾸며 보고 싶다면 → [충돌 제한 시각적으로 보는 데모 (RViz2)](VISUAL_DEMO.md)**. 이 데모는 실제 드라이버와 분리되어 있으며 허용 자세/요청 자세/보호 구/제한 상태를 보여줍니다.

ROS2 Humble용 첫 구현입니다. 실제 관절각 → Pinocchio FK → 공통 world의 보호 구 중심 XYZ → 좌우 모든 구 쌍의 표면 간격(중심거리 − 양쪽 반지름) → 정지 요청/명령 제한 순으로 동작합니다.

RB-Y1의 [27_collisions.py](https://github.com/RainbowRobotics/rby1-sdk/blob/300a585/examples/python/27_collisions.py)를 설계 참고로 사용했습니다. RB-Y1 예제는 로봇이 제공한 **링크 간 거리**를 50Hz로 읽고 3cm 미만이면 `cancel_control()`을 호출합니다. 이 코드는 Piper 관절 프레임의 점 거리를 직접 계산하므로 그 3cm를 재사용할 수 없습니다. RB-Y1 SDK나 제어 코드를 의존성으로 사용하지 않습니다.

## 현재 구현 범위

- 양팔 실제 관절 메시지를 이름으로 매핑하여 FK 계산. 누락/중복/NaN/범위 초과를 거부.
- 원본 STL의 링크별 경계 상자를 작은 셀로 나누고, 각 셀을 완전히 감싸는 구로 외형을 덮습니다. 베이스·link1~6·그리퍼 몸체·손가락 2개, 팔당 10개 링크/1,575개 구입니다.
- 손가락은 URDF의 전체 개폐 범위를 합친 고정 보호 영역을 사용합니다. 파지물은 포함하지 않습니다.
- `config/protection.yaml`의 `mesh_covers`에 링크 좌표계의 경계·셀 크기·반경 추가 여유를 저장합니다. `tools/generate_protection_spheres.py`로 원본 STL에서 재생성합니다. 기존 7점 사용자 설정도 지원하지만 현재 기본 모델은 아닙니다.
- 최소 표면 간격, 해당 쌍의 중심거리/이름/XYZ를 JSON으로 발행. RViz용 보호 구 MarkerArray 발행.
- 현재 상태와 실제→양팔 목표 관절 선형 보간 구간을 검사. 보간 예산 초과도 거부.
- 임계값 이하이면 정지 상태를 래치. 거리가 회복돼도 자동 재개하지 않음.
- 피드백/목표 지연, 양팔 수신시각 차이, 잘못된 입력도 명령 전달을 차단.
- 기본 **monitor 모드에서는 로봇 명령 publisher를 만들지 않음**.
- 선택적 **enforce 모드에서는 안전한 양팔 목표만 Piper 토픽으로 전달**. 차단 시 최신 유효 피드백을 한 번 캡처한 위치 유지 목표를 반복 전송. 피드백이 오래되면 위치 유지 명령도 보내지 않음.

[2차 MoveIt 직육면체 검사](../piper_moveit_guard/README.md)를 함께 사용하려면 `two_stage_guard.launch.py`를 실행합니다. 기존 `guard.launch.py` 단독 실행에는 MoveIt 검사가 없습니다. 보호 구는 원본 STL의 경계 상자 전체를 덮습니다. 손가락은 전체 개폐 범위를 포함하고, 파지물과 실제 부착물은 별도 모델링이 필요합니다. 경계 상자와 구는 실제 외형보다 보수적일 수 있습니다. 고정 간격의 보간 검사는 연속 충돌 검사의 보증이 아닙니다.

`stop_requested`는 **요청 신호**입니다. upstream Piper 드라이버는 이 신호를 자동 구독하지 않으며 이 패키지는 CAN EmergencyStop, disable 또는 reset을 호출하지 않습니다. 위치 유지 목표가 모터의 제동을 보장하지 않습니다. 프로세스 종료/피드백 상실 뒤에도 기존 목표가 남을 수 있어, 실물 사용에는 드라이버 쪽 독립 watchdog과 확인한 정지 동작을 연결해야 합니다. 실물 제동거리와 지연 검증 전에는 monitor 모드로 사용합니다. 모터/CAN에 연결한 시험은 수행하지 않았습니다.

## 설치 및 빌드

ROS2 Humble 환경의 Python 3.10과 Pinocchio가 필요합니다. 아래는 시스템 Python 패키지를 바꾸지 않는 방식이며, 개발 검증에는 Pinocchio 2.7.0을 사용했습니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/Robocup-Home-2027_VLA_Tutorial
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install pin==2.7.0 scipy==1.10.1
python /usr/bin/colcon build --symlink-install --packages-select piper_dual_description piper_collision_guard
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch piper_collision_guard guard.launch.py
```

`python3 -m venv`가 없다면 Ubuntu의 python3-venv 패키지가 필요합니다. 새 터미널에서도 ROS → 가상환경 → 작업공간 순서로 활성화합니다. 기본 URDF는 함께 설치한 piper_dual_description에서 찾습니다. 다른 모델은 YAML의 urdf_path로 지정할 수 있습니다.

## 토픽과 제어 경로

| 방향 | 토픽 | 형식 |
|---|---|---|
| 피드백 입력 | /joint_states_left, /joint_states_right | sensor_msgs/msg/JointState |
| 텔레옵 목표 입력 | /teleop/left_target, /teleop/right_target | sensor_msgs/msg/JointState |
| 명령 출력 (enforce만) | /joint_ctrl_cmd_left, /joint_ctrl_cmd_right | sensor_msgs/msg/JointState |
| 상태 출력 | /collision_guard/status | std_msgs/msg/String (JSON, transient local) |
| 정지 요청 | /collision_guard/stop_requested | std_msgs/msg/Bool (transient local) |
| 시각화 | /collision_guard/markers | visualization_msgs/msg/MarkerArray |

피드백과 목표는 각각 `joint1`~`joint6` 또는 해당 팔의 `left_joint1`/`right_joint1` 이름을 사용합니다. `gripper`(또는 joint7)는 **Piper 드라이버 관례의 전체 개구 폭(m)**입니다. Pinocchio 손가락 한쪽 이동량을 이 필드에 넣지 않습니다. 입력은 관절 rad, 그리퍼 m이며 현재 ROS 시간의 유효한 header.stamp가 반드시 필요합니다. 양팔 시계는 동일 시간 기준이어야 합니다. 1차 거리 검사는 손가락 전체 개폐 영역을 고정 보호 구로 덮으므로 개구 폭에 따라 줄어들지 않습니다.

상태 확인:

```bash
ros2 topic echo /collision_guard/status
```

RViz에서 Fixed Frame을 `world`로 두고 MarkerArray에 `/collision_guard/markers`를 추가합니다. 반투명 구 크기는 설정한 보호 반지름을 표시합니다. 반지름 0의 계산 테스트용 점만 최소 2mm로 표시합니다. URDF 표시/TF는 별도의 robot_state_publisher 구성이 필요합니다.

## 임계값과 재개

`config/protection.yaml`은 원본 메시 경계 상자를 덮는 보호 구를 생성합니다. 일반 링크 셀은 최대 40mm, 그리퍼 몸체 셀은 최대 8mm이며 각 구 반지름은 셀 대각선의 절반 + 1mm입니다. 구마다 반지름이 다르므로 고정 중심 거리 10cm를 사용하지 않습니다. 정지 기준은 보호 구 표면 간격 `stop_clearance_m=0.02`(20mm 이하)입니다. 1mm는 모델 추가 여유이며 실측 오차/제동 검증값은 아닙니다.

양팔 0°에서는 보호 구 최소 간격 약 22.16mm로 통과합니다. 왼팔 J1만 +15°인 문제 자세는 약 -23.45mm로 차단합니다. 원본 메시 실제 최소 간격은 해당 자세에서 약 1.35mm입니다. 보호 구는 경계 상자를 감싸므로 실제 접촉보다 일찍 제한할 수 있습니다.

정지 후 재개 기준은 기존대로 40mm 초과입니다. 따라서 0° 자세(약 22.16mm)로 돌아가는 것만으로 실기 guard의 reset 조건을 만족하지는 않습니다. 감시 모드에서 재개 간격을 확인하고, 실기 검증을 마친 재개 절차를 사용하세요. 데모는 안전 영역 복귀 시 자동으로 따라갑니다.

`mesh_covers`의 경계와 셀 크기를 바꾸면 보호 영역이 달라집니다. 생성기와 원본 메시를 기준으로 외형 포함 여부를 다시 검증합니다. 정지 여유에는 전체 지연·제동거리·실물 오차를 별도로 반영해야 합니다.

거리·좌표·제동을 확인한 뒤에만 명시적으로 enforce를 켭니다:

```bash
ros2 launch piper_collision_guard guard.launch.py enforce:=true geometry_confirmed:=true
```

geometry_confirmed는 보호 구와 정지 여유를 검증했음을 표시하는 시작 설정입니다. 기본 false에서는 enforce 시작을 거부합니다.

enforce는 upstream Piper **위치 제어** 토픽 전달용이며 MIT/토크 제어용이 아닙니다. 안전한 목표도 YAML의 driver_speed_ratio로 전달합니다. upstream 드라이버는 velocity[6]을 전역 속도 비율로 해석하므로 일반적인 관절별 속도 명령을 전달하지 않습니다. gripper effort도 설정값으로 재구성합니다.

정지 래치 해제:

```bash
ros2 service call /collision_guard/reset std_srvs/srv/Trigger '{}'
```

신선한 실제 피드백과 재개 거리 초과가 모두 필요합니다. 해제 시 이전 목표를 모두 폐기하며 양팔의 새 목표를 받아야 전달합니다. 위험한 목표는 다시 차단됩니다. 운전자가 퀘스트 기준점을 다시 맞춘 뒤 새 목표를 보내는 방식이 좋습니다.

목표/피드백/URDF 경로 등 설정은 시작 시 읽습니다. YAML을 수정한 다음 노드를 재시작하세요.

## 기존 드라이버에 연결할 때

퀘스트 IK는 /teleop/*_target으로만 발행하고, /joint_ctrl_cmd_*로 직접 발행하지 않게 바꿉니다. Cartesian /pos_cmd_*와 다른 SDK 직접 호출도 우회 경로가 되지 않게 정리해야 합니다. 이 노드는 다른 명령 publisher를 감지하면 자신의 전달을 차단하지만 외부 publisher 자체를 막을 권한은 없습니다.

권장 첫 단계는 **드라이버 실제 상태를 읽는 monitor 모드**입니다. 아래 테스트는 별도 ROS domain의 가짜 메시지만 사용하고 실제 로봇을 제어하지 않습니다.

## 테스트

```bash
PYTHONPATH="$PWD/piper_collision_guard:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper_collision_guard/test/test_distance.py -q
```

원본 양팔 베이스 간격/FK, 메시지 매핑, 가장 가까운 교차 인덱스 쌍, 래치·재개·지연, 모델 범위, 경로 중간 충돌과 보간 예산을 검사합니다.

ROS 통합 테스트는 실제 드라이버 없이 별도 domain에서 수행합니다:

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 RUN_ROS_GUARD_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest piper_collision_guard/test -q
```
