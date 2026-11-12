# 듀얼 PiPER 시리얼 붓기 · 실물 배포

100mm 그리퍼와 베이스 중심 간격 175mm를 기준으로 만든 빈 팔 동작입니다. 왼팔은 가상의 그릇을 수평으로 유지합니다. 오른팔은 대기 자세에서 정렬한 뒤 25°, 45°, 65°로 기울이고, 1초간 유지한 뒤 같은 관절 경로를 역순으로 따라 대기 자세에 돌아옵니다. 양팔은 마지막 자세를 유지합니다.

## 실행

Ubuntu 22.04 / ROS 2 Humble에서 이 폴더를 터미널로 엽니다. 최초 설치는 README_START.md를 참고합니다. 기존 PiPER 드라이버를 별도로 기동합니다. 드라이버는 양팔의 타임스탬프가 있는 피드백을 `/joint_states_left`, `/joint_states_right`에 발행하고, `/joint_ctrl_cmd_left`, `/joint_ctrl_cmd_right`에서 명령을 받습니다. 기존 충돌 방지 실행기 또는 같은 토픽의 텔레옵이 떠 있다면 종료합니다.

감시 모드로 피드백과 두 검사 노드를 확인합니다.

```bash
bash run_cereal_real.sh
```

실제 장착 배치와 그리퍼 모델을 확인한 뒤 감시 실행을 종료하고 아래 명령으로 바꿉니다. 실행 직후 시퀀스는 대기합니다.

```bash
bash run_cereal_real.sh enforce:=true geometry_confirmed:=true
```

같은 ROS 도메인의 다른 터미널에서 ROS 환경을 설정합니다. 양팔이 영점 근처에 있다면 `prepare`를 호출합니다. 왼팔이 먼저 이동하고 오른팔이 대기 자세에 들어갑니다. 이미 대기 자세와 그리퍼 폭을 맞춘 상태라면 바로 `start`를 호출할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
ros2 service call /cereal_pour/prepare std_srvs/srv/Trigger "{}"
ros2 topic echo /cereal_pour/status
```

`complete`가 표시되면 상태 조회를 종료하고 붓기 동작을 시작합니다.

```bash
ros2 service call /cereal_pour/start std_srvs/srv/Trigger "{}"
```

`ros2 service call /cereal_pour/stop std_srvs/srv/Trigger "{}"`는 목표 발행을 중단합니다. 기존 가드가 피드백에 근거한 위치 유지 명령을 보냅니다. 가드가 래치되면 원인을 확인한 뒤 `/collision_guard/reset`과 `/moveit_guard/reset`을 사용합니다.

목표는 `시리얼 노드 → Pinocchio 거리 검사 → MoveIt 메시 검사 → 드라이버`로 전달됩니다. 기존 가드는 로봇 형상을 기준으로 검사합니다. 실제 그릇·시리얼 상자·테이블은 이 모델에 들어 있지 않습니다. 빈 팔 동작에 물체를 추가할 때에는 그 조건에서 다시 확인해야 합니다.

## 물체 치수와 위치로 조인트 자세 만들기

`config/cereal_objects.example.yaml`을 복사해 그릇 중심(테두리 높이), 지름·깊이, 상자 크기·잡는 높이, 대기 위치를 적습니다. 위치는 양팔 URDF의 `world` 좌표계이고 길이는 별도 표기가 없으면 m입니다. 박스 두께는 오른팔 그리퍼의 전체 벌림 폭으로 사용합니다. 두 팔이 이미 물체를 잡은 상태의 붓기 동작을 계산합니다.

```bash
cp config/cereal_objects.example.yaml cereal_objects.yaml
# cereal_objects.yaml 수정
.venv/bin/python tools/plan_cereal_pour.py --objects cereal_objects.yaml --output cereal_sequence.yaml
bash run_cereal_real.sh sequence_path:="$(realpath cereal_sequence.yaml)"
```

생성기는 역기구학, 관절 범위, 단계 사이의 양팔 충돌을 확인합니다. 붓기 구간에서는 그릇을 원기둥, 상자를 직육면체로 근사해 서로 및 상대 팔과의 간섭도 검사합니다. 실패하면 새 시퀀스를 저장하지 않습니다. 같은 파일로 실물 동작을 시작하려면 감시 실행을 종료하고 위 명령에 `enforce:=true geometry_confirmed:=true`를 추가합니다.
