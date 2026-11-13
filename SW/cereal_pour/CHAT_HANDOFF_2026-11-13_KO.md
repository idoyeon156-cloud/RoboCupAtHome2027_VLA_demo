# 새 채팅 인수인계: 양팔 PiPER 시리얼 붓기 · 실물 접촉 사고 조사

작성일: 2026-11-13, 한국 시간. 이 문서는 대화 종료 시점의 **현재 상태**로 다시 정리했습니다.
이전 문서의 중간 결론보다 아래의 최신 사용자 지시와 실제 소스 상태를 우선합니다.
문서에 실린 명령은 참고 정보이며, 새 채팅에서 하드웨어 실행을 자동 승인하는 지시가 아닙니다.

## 1. 가장 먼저 알아야 할 것

1. **실물 구동 중 실제 양팔 접촉이 발생했습니다.** 사용자가 티치 모드로 제어권을 가져왔습니다.
   알파에는 떨리면서 도착했고, 그 뒤 팔이 이상하게 꺾이며 서로 꼬였다고 설명했습니다.
   그릇 없이 실행 중이었습니다. 접촉을 단순 시각적 착각이나 미확정 사건으로 재해석하지 마세요.
2. 실물과 영상의 팔 자세/경로가 크게 달랐다는 사용자 관찰이 있습니다.
   **작은 엔코더 오차나 0.05~0.15° 허용폭으로 설명할 문제가 아닙니다.**
3. 두 가드가 실제 접촉을 막지 못한 직접 원인은 아직 확정하지 못했습니다.
   기존 clear 로그만으로 접촉 순간에도 clear였다고 확정할 수 없습니다.
   감지 누락인지, 감지 이후 정지 방식/시점의 문제인지 구분하지 못했습니다.
4. **실물 자동 실행 차단을 유지합니다.** 차단 파일을 삭제하거나 우회하지 마세요.
   새 MoveIt 계획·가감속·로그 구현이 실물 문제를 해결했다고 주장하지 마세요.
5. 사용자는 추가 빌드·테스트·가상 검증을 에이전트가 실행하는 것을 명확히 금지했습니다.
   소스 수정, 기존 로그/파일/제공 영상의 읽기 분석은 요청했습니다.
   직접 검증하겠다며 로봇/시뮬을 실행하거나 가드 리셋을 하지 마세요.
6. CAN 좌우 고정 가능성을 조사했지만 **사용자는 CAN 변경을 뒤로 미루고 팔 동작과 접촉 검출 문제부터 해결하라고 했습니다.**
   CAN을 임의 교환하거나 이 가설에 다시 매달리지 마세요.

## 2. 사용자가 원하는 최종 동작과 시뮬

- 명령 한 번으로 **영점 → 알파 → 그릇/시리얼 파지 → 붓기 → 내려놓기 → 영점 복귀**.
- 알파에서 사용자가 다시 start를 입력하는 두 단계 흐름을 원하지 않습니다.
- 알파 J1~J6: **양팔 모두 [0, 90, 0, 0, -70, 0]°**. J5는 +70°가 아닙니다.
- 사용자가 먼저 시뮬에서 동작을 보고 실물로 넘어갑니다.
- 시뮬도 **MoveIt이 충돌 조건을 적용해 경로를 생성**하고, 실행 중 **거리 가드와 MoveIt 메시 가드**가 차단해야 합니다.
- 시간표대로 재생하는 애니메이션을 실제 가드가 통과한 동작과 같다고 설명하면 안 됩니다.
- 현재 제일 중요한 문제는 **로봇 외곽끼리/지그·환경과 실제 접촉했는데 차단되지 않은 이유**입니다.

## 3. 마지막 대화에서 바로잡은 내용 — 중요

사용자 마지막 지적:

> 파이퍼 전체 표면을 메시로 따는 것이지, 부품 조립 단면을 서로 장애물처럼 검사하는 것이 아니다.
> 그렇게 하면 조립 부위에서 내부 충돌만 생긴다. 실제 충돌을 놓친 것이 문제인데 왜 다른 부분을 건드리느냐.

### 잘못 진행했던 변경

- 에이전트가 같은 팔 전체 제외를 제거하고 고정/인접 링크만 제외하도록 자기충돌 범위를 확대했습니다.
- 그 결과 사용자가 새 시뮬을 실행했을 때 다음 오류로 경로 계획이 시작 자세에서 중단됐습니다.

```text
Planning failed; sequence not replaced: MoveIt start state rejected:
MoveIt Bullet/feasibility rejected state:
left_base_link/left_link2, right_base_link/right_link2
```

- 공식 PiPER SRDF의 base_link/link2 `Never` 제외를 근거로 이 한 쌍을 제외하려 했다가 되돌렸습니다.
- 이어서 무여유 원본 CAD로 베이스/link2만 검사하는 보조 모델과 C++ helper를 잠시 추가했습니다.
- **이것도 사용자 지적 후 전부 철회했습니다.** 같은 팔 조립 내부 검사 확대는 본래 사건을 해결하지 못하고 별도 오류를 만들었습니다.

### 현재 실제 소스 상태

- `collision_model.py::load_semantic`은 **같은 팔의 조립 내부 링크 쌍을 제외하는 원래 범위로 복원**했습니다.
- 같은 팔 충돌 형상 쌍은 기존 SRDF 제외를 유지하며, 누락된 동일 팔 쌍은 `SameArmAssemblyInternal`로 제외합니다.
- 반대 팔 사이 또는 로봇–환경 사이의 SRDF 제외는 제거합니다. 외부 검사 범위를 넓히는 방향이지 외부 충돌을 허용하는 변경이 아닙니다.
- `load_nominal_shoulder_model`, `robot_description_nominal` 관련 연결, C++ `ShoulderCollision` 호출은 제거했습니다.
- `shoulder_collision.h` 파일은 삭제했습니다. `PiperReferenceBaseLink2` 개별 복원도 현재 없습니다.
- 충돌 메시 크기나 전체 5mm 여유를 줄이는 변경은 하지 않았습니다.
- MoveIt 계획, 두 가드, 가상 피드백 RViz, 로그 보강, 가감속, 손목 선호 변경은 남아 있습니다.
- 복원 이후 에이전트 빌드/테스트/시뮬/실물 실행은 하지 않았고, 사용자 재실행 결과도 아직 없습니다.

**주의:** 현재 구현은 가동 링크별 메시를 사용하는 URDF 모델입니다. 같은 팔 조립 내부를 외부 충돌로 보고하지 않는 정책으로 복원한 것이며, 팔 전체를 하나의 불변 STL로 합친 구현이 아닙니다. 새 자기충돌 모델을 설계하는 일을 본래 문제와 혼동하지 마세요.

## 4. 작업 위치와 환경

- 저장소: `/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo`
- 주 작업 폴더: `/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour`
- 프로젝트 가상환경: `/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/.venv`
- ROS 2 Humble: `/opt/ros/humble/setup.bash`
- 설치 드라이버: `/home/doyeonlee/piper_ros`
- SDK: `/home/doyeonlee/piper_sdk`
- 제공 시뮬 영상: `/home/doyeonlee/바탕화면/Cereal_Pour_alpha.webm`
- 여러 폴더에 사용자 변경/이전 작업 변경이 섞여 있습니다. `git reset`/일괄 되돌리기로 지우지 마세요.
- 최신 작업은 로컬 미커밋/미추적 파일입니다. 커밋·푸시·PR을 만들지 않았습니다.
- 기존 ZIP은 최신 소스와 일치하지 않습니다. 패키징은 최근 변경 이후 실행하지 않았습니다.

## 5. 실물 실행 차단

차단 파일:

`/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/REAL_MOTION_HOLD_KO.md`

- 드라이버 자동 활성화 래퍼와 실물 패키지 래퍼가 이 파일이 있으면 종료합니다.
- 제어 도구의 ready/home/prepare/start/run도 차단하며 check/status/stop은 허용합니다.
- 실물 cereal launch factory에도 차단 확인을 넣었습니다.
- 차단 파일은 **이미 실행된 노드나 다른 폴더의 SDK를 종료하지 않습니다.** 하드웨어 비상정지가 아닙니다.
- 사고 후 현재 물리 자세/모터 제어권/프로세스 상태를 확정하지 마세요. 마지막에 양팔이 영점으로 돌아왔다고 재확인한 기록은 사고 이후에는 없습니다.

## 6. 기존 사고 로그와 확인된 사실

핵심 로그 폴더:

`/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/meshes_real`

| 파일 | 내용 |
| --- | --- |
| [python_19541_1794548877569.log](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/meshes_real/python_19541_1794548877569.log>) | 사고 당시 cereal 실행기, motion_progress와 최종 정지 |
| [python_19537_1794548877555.log](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/meshes_real/python_19537_1794548877555.log>) | 거리 가드 |
| [moveit_guard_19539_1794548877305.log](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/meshes_real/moveit_guard_19539_1794548877305.log>) | MoveIt 메시 가드 |
| [2026-11-13-14-47-57-184788-doyeon-galaxy-19374/launch.log](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/meshes_real/2026-11-13-14-47-57-184788-doyeon-galaxy-19374/launch.log>) | 실행 프로세스/인자/종료 |
| [python_17930_1794547413408.log](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/meshes_real/python_17930_1794547413408.log>) | 이전 실행: 느린 알파와 bowl_open 시간 초과 |

실행 당시 남아 있던 파라미터 파일(현재도 있는지는 읽기 확인 필요):

- `/tmp/launch_params_qez88bqz`: MoveIt 모델/가드 설정.
- `/tmp/launch_params_jsp5anu1`: 거리 가드 설정.
- `/tmp/launch_params_hbn00c1k`: cereal 설정.

확인 내용:

- 두 가드는 enforce였고, MoveIt은 convex_meshes=true였습니다.
- 로봇+고정 기둥의 충돌 링크 23개, convex 부품 73개가 로드됐습니다.
- 그리퍼/날개/마운트가 모델에서 통째로 빠졌다는 증거는 없었습니다.
- 좌우 팔 사이의 전체 제외는 없고, C++도 좌우 팔 쌍을 검사하도록 ACM을 설정합니다.
- alpha는 epoch 1794548899.9106~1794548910.5791, 약 10.67초.
- bowl_open은 약 1.80초. 그리퍼 측정 폭 29mm, 목표 30mm로 다음 단계에 진입했습니다.
- bowl_pregrasp 왼팔 목표는 `[36.512,120.344,-50.064,62.173,-56.274,-23.172]°`, 오른팔은 알파 유지였습니다.
- **J4 0→62.173°, J6 0→-23.172°의 큰 손목 회전 자체가 기존 IK 목표에 있었습니다.**
  미세 엔코더 오차가 이 목표를 만들어 낸 것이 아닙니다. 단, 이것만으로 실제 접촉의 직접 원인을 확정할 수 없습니다.
- 마지막 왼팔 명령은 `[25.812,111.452,-35.393,43.953,-60.297,-16.382]°`,
  피드백은 `[25.781,111.425,-35.304,44.017,-60.307,-16.384]°`였습니다.
- 1초 기록 표본의 최대 명령-피드백 차이:
  - alpha: 왼팔 1.446°, 오른팔 1.487°.
  - bowl_pregrasp: 왼팔 1.173°, 오른팔 1.720°.
  - 표본 사이의 접촉/순간 오차를 배제하는 수치는 아닙니다.
- 오른팔 J4는 목표 0°인데 후반 피드백이 -1.72°로 변했습니다. 티치 개입/외력/기타 원인은 로그만으로 구분되지 않습니다.
- 최종 cereal 정지는 오른팔 J3 +0.086°가 상한 0°와 허용폭 0.05°를 넘은 피드백 오류였습니다.
  이는 충돌 검출을 의미하지 않습니다.
- 과거 가드는 최초 stop 전환을 파일에 항상 기록하지 않았습니다. cereal의 1초 주기 clear 표본만으로 접촉 순간의 상태를 단정하지 마세요.
- 가드의 정지는 신선한 피드백이 있으면 위치 유지 명령을 발행하는 구조입니다.
  이미 전송된 제어기 이동의 즉시 취소나 토크 차단/하드웨어 비상정지를 보장하지 않습니다.

## 7. 시뮬과 실물 차이에 대해 확인한 범위

- 제공 영상은 약 71초입니다. 약 9초에서 화면 오른쪽 파란 팔이 그릇 쪽으로 움직이며 주황 팔은 알파를 유지합니다.
- 코드 색상에서 파랑은 논리 left입니다. 이를 물리 CAN 좌우 확인으로 해석하지 마세요.
- 현재 경로와 Git 기준 경로의 49개 waypoint 이름/양팔 각도/개방 폭/파지 물체는 같았습니다.
- 영상 당시 RViz가 읽은 파일의 지문은 기록되지 않아 영상과 실물의 수치적 동일성은 확정하지 못했습니다.
- 양팔 J1~J6 origin xyz/rpy 및 axis는 설치된 공식 PiPER URDF와 수치상 일치했습니다.
  이것은 파일끼리의 일치이며 실제 베이스 설치·교정 영점·그리퍼 조립과의 일치를 증명하지 않습니다.
- 드라이버는 `joint1`~`joint6`을 이름으로 SDK JointCtrl(j1,...,j6)에 전달합니다.
  이 경로에서 부호 반전/관절 순서 교환은 발견되지 않았습니다.
- `/joint_states_left/right`는 `GetArmJointMsgs()`의 센서 관절값입니다.
  `GetArmJointCtrl()`의 명령 echo는 별도 `/joint_states_ctrl_left/right`입니다.
- 기존 RViz는 시간 보간된 예정 관절값을 표시했고, 실물은 MOVE J와 피드백 추종을 거쳤습니다.
  표시/실행 구조 차이는 확정됐지만 큰 정적 자세 차이의 원인이 이것이라고 확정할 수는 없습니다.
- 실제 베이스/도구 좌표 정합과 설치 형상이 미확인입니다. CAN뿐 아니라 이 부분도 아직 가설이지 확정 원인이 아닙니다.

## 8. 현재 소스에 남아 있는 MoveIt 계획·시뮬 구성

### 경로 생성

1. 물체 좌표로부터 Pinocchio FK + SciPy least_squares 수치 IK로 목표 관절각 후보를 구합니다.
2. native MoveIt PlanningPipeline + OMPL RRTConnect가 관절 경로를 생성합니다.
3. MoveIt Bullet 메시 검사와 Python feasibility callback의 거리 가드/물체/파지 물체 조건을 계획 중 적용합니다.
4. 반환 경로의 실행 보간 구간도 검사합니다. 자체 RRT/무검사 직선 경로 fallback은 제거했습니다.
5. IK 불가/충돌/시간 초과 때 기존 출력 파일을 교체하지 않고 실행을 중단하도록 했습니다.
6. 현재 자기충돌 정책은 3절의 **조립 내부 제외, 양팔/환경 검사**입니다.

계획 프로세스는 전용 ROS domain 191과 고유 서비스 이름을 사용하며, 드라이버 명령 publisher가 없습니다.
수치 IK가 MoveIt의 Pinocchio 플러그인이라는 뜻은 아닙니다.
실행은 MoveIt 경로의 waypoint를 자체 실행기로 보간하는 구조이며, MoveIt FollowJointTrajectory 컨트롤러에 직접 넘기는 구현은 아닙니다.

### 시뮬 실행

`시퀀스 실행기 → 거리 가드(enforce) → MoveIt 메시 가드(enforce) → 이상적 가상 드라이버 → RViz 피드백`

- 주 제어 토픽과 서비스는 `/cereal_sim` 아래로 remap합니다.
- RViz는 통과한 최종 명령의 가상 피드백을 표시하도록 변경했습니다.
- 최초 영점에서 준비/가드 리셋을 한 번 수행하고 전체 순서를 한 번 실행합니다. 중간 자동 리셋/재시도는 없습니다.
- 상태 만료/가드 잠금/시퀀스 오류 시 가상 팔 진행을 중단합니다.
- 정상 완료 표시는 `SIMULATION COMPLETE`, 화면 `IDEAL GUARDED SIM: complete`입니다.
- 제어기 관성·모터 추종·실물 오버슈트·접촉 힘은 모델링하지 않습니다.
- 레거시 `moveit_box_demo.launch.py cereal:=true`의 시간표 재생은 거부하고 새 래퍼로 안내하도록 변경했습니다.

사용자가 직접 실행할 명령(에이전트 자동 실행 금지):

```bash
cd "/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour"
bash run_cereal_rviz.sh
```

이 명령은 빌드와 필요한 MoveIt 재계획을 수행합니다. CAN/실물 드라이버는 필요하지 않습니다.
사용자는 이 흐름에서 MoveIt 시작 상태 오류를 한 번 보고했습니다. 그 실행은 전체 동작 성공이 아닙니다.
**최신 조립 내부 제외 복원 이후 실행 성공은 아직 확인하지 못했습니다.**

### 시뮬 경로 고정

- 실행 경로를 작업 폴더의 `model/simulation_sessions/<실행ID>/sequence.yaml`에 고정 복사합니다.
- 전체 완료 시 receipt와 `model/last_completed_simulation.json`에 경로/소스/모델/속도 지문을 기록합니다.
- 향후 실물 실행은 동일 완료 경로를 요구하도록 했지만, 완료 receipt가 실물 차단 파일을 해제하지는 않습니다.

## 9. 팔 동작·드라이버 변경 — 소스 상태, 효과 미확인

- 수치 IK 해에서 손목 J4/J6 변화에 기본 가중치 3을 주어 작은 손목 회전을 우선합니다.
- MoveIt 손목 경로는 시작~목표 범위에 기본 15° 여유를 허용합니다. 목표로 필요한 62° 회전 자체를 무조건 15°로 제한한 것은 아닙니다.
- 같은 위치/파지 방향을 유지합니다. 적절한 작은 회전 해가 실제 존재하는지는 새 실행으로 확인되지 않았습니다.
- 목표 보간을 `SegmentMotion`의 공통 스칼라 진행률로 바꾸어 검사한 직선 경로 위에서 가감속하도록 했습니다.
- 기본 정상 진행 목표 가속도는 관절 20°/s², 그리퍼 20mm/s²입니다. 하드웨어 안전 보증값이 아닙니다.
- 목표 속도는 일반 관절 15°/s × scale 1.5 = 22.5°/s, 그리퍼 20mm/s입니다.
- J5가 알파 끝점 마지막 5°에 접근할 때 1°/s 제한을 유지하고 앞에서 감속하도록 했습니다.
- OMPL의 직선상 중복 표본만 줄여 표본마다 정착 대기하는 문제를 피하며 코너는 보존합니다.
- 실제 드라이버는 매 목표마다 MotionCtrl_2(CAN, MOVE_J)를 호출합니다.
  기존 enable flag는 티치 전환만으로 갱신되지 않아, 사용자가 티치로 권한을 가져온 뒤에도 CAN 제어 모드를 다시 요청할 수 있는 구조였습니다.
- 로컬 driver overlay에 모드/티치/오류/상태 신선도 검사를 추가하고 제어권 상실 후 위치 명령을 잠그도록 했습니다.
  이것은 이미 보낸 이동 취소/비상정지가 아닙니다. 원본 piper_ros나 현재 프로세스에는 직접 적용하지 않았습니다.

## 10. 이전 피드백/타임아웃 조정

- J5 -70° 목표를 유지합니다. 측정값만 J5 경계 초과 0.15°, 다른 관절 0.05° 이내에서 정규화합니다.
- 이 값은 검증된 엔코더 사양이 아니라 관측값에 근거한 운용 허용폭입니다.
- 닫힌 그리퍼 센서 [-0.25mm,0)만 0으로 정규화합니다. 목표값과 URDF 한계는 엄격히 검사합니다.
- 원본 드라이버의 근사 변환 0.017444 / 57324.840764를 정확한 math.pi 변환으로 바꾸는 overlay를 만들었습니다.
  근사 상수 차이는 약 0.05%로, 큰 자세 차이의 설명이 아닙니다.
- SDK의 0도 **이동 명령**과 영점 **교정**은 다릅니다. 사용자가 새 교정을 했다고 가정하지 마세요.
- 그리퍼 단계 최소 제한 20초, 목표 방향으로 0.25mm 이상 진행이 3초간 없으면 정지하도록 했습니다.
- `run`은 초기 확인/두 가드 리셋 후 영점에서 전체 시퀀스를 요청합니다. 중간 재리셋은 없습니다.
- 이런 조정으로 실제 접촉 문제가 해결됐다고 해석하지 마세요.

## 11. 로그 보강

새 코드가 실행될 때 기록할 폴더:

`/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/log/<mode>/diagnostics/<실행ID>/`

- `cereal_pour.jsonl`: 단계 전환, 좌우로 구분한 목표/명령/측정, 명령 변화율, 두 가드 상태.
- `piper_collision_guard.jsonl`: 최초 stop/unlock/reset, 원본 피드백/시각/오류, 목표, 가까운 쌍과 최소 거리.
- `moveit_guard.jsonl`: 최초 stop/unlock/reset, 충돌 링크와 겹침 깊이, 현재/예측 검사 자세, 원본/정규화 피드백과 목표.
- final gate 출력과 SDK command echo를 수신 시 관찰 기록합니다. 명령 echo와 센서 피드백을 구분합니다.
- PiperStatusMsg가 환경에 있으면 제어/티치/오류 상태 변화를 기록합니다. 없으면 unavailable을 명시합니다.
- 전환은 발생 시, 상세 표본은 최대 20Hz로 기록하며 monotonic_ns로 같은 PC의 순서를 합칩니다.
- 파일 기록 실패는 logger에 표시합니다. 이 로깅이 하드웨어 정지 기능을 대신하지 않습니다.
- 계획 전용 로그는 작업 폴더 `log/planning/<계획ID>.log`입니다.
- **기존 사고 로그에는 이 새 JSONL 기록이 없습니다.** 소스 추가로 과거 접촉 순간을 복구할 수는 없습니다.

파일만 읽는 도구:

```bash
cd "/home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour"
python3 tools/read_session_logs.py
python3 tools/read_session_logs.py --samples --last 10
python3 tools/read_session_logs.py --json --last 5
```

## 12. 베이스/link2 분석 자료의 의미와 한계

경로: 작업 폴더 `analysis/base_link2_zero_2026-11-13/`.

- 기존 파일의 영점 자세에서 수행한 정적 메시 좌표 계산입니다. ROS/로봇/새 동작 시뮬은 실행하지 않았습니다.
- 높이 범위 차이 3.279mm를 실제 표면 최소 간격처럼 처음 설명한 것은 잘못이었습니다.
- 후속 원본 삼각형 표면 거리 계산은 모델상 6.546mm, 원본 전체 볼록껍질은 약 5.168mm,
  확장된 link2 첫 부분과 베이스 볼록 모델은 약 4.771mm 겹침으로 계산됐습니다.
- **모델 좌표 계산이지 실물 측정이 아닙니다.** 사용자는 실물에서 그렇게 가깝지 않다고 지적했습니다.
- 이 수치로 실물 조립/영점이 일치한다고 주장하거나 사용자 관찰을 부정하지 마세요.
- 이후 사용자는 조립 내부 검사 확대 자체가 본래 문제에서 벗어났다고 지적했고 그 변경을 철회했습니다.
  이 정적 분석을 실물 접촉 사고의 해결책이나 다음 최우선 과제로 삼지 마세요.

## 13. CAN 확인 결과 — 현재는 변경 보류

현재 시스템에 PiPER CAN 이름 고정 udev/systemd 규칙은 없었습니다.
USB 어댑터의 서로 다른 고유 시리얼을 확인했습니다.

| 읽기 확인 시 인터페이스 | USB 시리얼 |
| --- | --- |
| can0 | 004E00235246571520393733 |
| can1 | 003400155343570F20363330 |

- bytewerk candleLight USB to CAN, VID:PID 1d50:606f, gs_usb.
- 시리얼 기반 이름 고정은 가능합니다. 규칙 설치·CAN 이름/상태 변경은 하지 않았습니다.
- 현재 인터페이스 번호는 사고 당시의 배선/물리 좌우 확인이 아닙니다.
- 래퍼 기본값은 논리 left=can0, right=can1이며 물리 위치를 자동 검출하지 않습니다.
- USB 인식 순서 변경 가능성은 있으나 사고 때 실제 좌우가 바뀌었다는 증거는 없습니다.
- **사용자 지시로 이 작업은 보류했습니다.**

## 14. 주요 소스 파일

아래 링크는 실제 파일의 절대 경로입니다.

| 파일 | 역할 |
| --- | --- |
| [piper_collision_guard/piper_collision_guard/collision_model.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/collision_model.py>) | 외부 충돌 범위/semantic, 메시 URI 해석. 최신 롤백의 핵심 |
| [piper_collision_guard/piper_collision_guard/cereal_planning.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/cereal_planning.py>) | 목표 IK/객체 조건/경로 연결/출력 해시 |
| [piper_collision_guard/piper_collision_guard/moveit_backend.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/moveit_backend.py>) | Python feasibility 서비스와 native MoveIt 계획 서비스 연결 |
| [piper_moveit_guard/src/cereal_moveit_planner.cpp](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_moveit_guard/src/cereal_moveit_planner.cpp>) | MoveIt PlanningPipeline + OMPL/Bullet, 계획/상태 검사 서비스 |
| [piper_collision_guard/piper_collision_guard/guarded_launch.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/guarded_launch.py>) | 실물/시뮬 공통 두 가드 파이프라인과 remap |
| [piper_moveit_guard/launch/cereal_guarded_rviz.launch.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_moveit_guard/launch/cereal_guarded_rviz.launch.py>) | 새로운 가드 연동 RViz 진입점 |
| [piper_collision_guard/piper_collision_guard/simulation_driver.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/simulation_driver.py>) | CAN 없는 이상적 피드백/일회 전체 실행 조정 |
| [piper_collision_guard/piper_collision_guard/cereal_preview.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/cereal_preview.py>) | 새 경로에서는 실제 가상 피드백으로 물체 표시 |
| [piper_collision_guard/piper_collision_guard/cereal_pour.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/cereal_pour.py>) | 단계 실행, 추종/시간 제한, 좌우 로그 |
| [piper_collision_guard/piper_collision_guard/segment_motion.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/segment_motion.py>) | 검사한 관절 경로 위의 목표 가감속 |
| [piper_collision_guard/piper_collision_guard/node.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/node.py>) | 거리 가드 |
| [piper_moveit_guard/src/moveit_guard.cpp](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_moveit_guard/src/moveit_guard.cpp>) | 최종 메시 충돌 가드와 위치 유지 명령 |
| [piper_collision_guard/piper_collision_guard/diagnostics.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/diagnostics.py>) | JSONL 기록 |
| [piper_collision_guard/piper_collision_guard/plan_snapshot.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/piper_collision_guard/piper_collision_guard/plan_snapshot.py>) | 실행 경로/소스/모델 지문 고정 |
| [tools/prepare_driver_overlay.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/tools/prepare_driver_overlay.py>) | 정확한 각도 변환 + 제어권 상실 잠금을 넣는 로컬 복사본 생성 |
| [tools/real_control.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/tools/real_control.py>) | 읽기 확인/명시적 서비스 요청, 실물 차단 확인 |
| [tools/read_session_logs.py](</home/doyeonlee/문서/ChatGPT/VLA 공부/RoboCupAtHome2027_VLA_demo/SW/cereal_pour/tools/read_session_logs.py>) | ROS/CAN 없이 새 로그 읽기 |
| `run.sh` | 환경/빌드/계획/실행. 실제 호출은 사용자가 하도록 안내 |

설치/형상 입력:

- 물체 설정: 작업 폴더 `config/cereal_task.yaml`.
- 모델/설치 설정: 작업 폴더 `config/robot.yaml`.
- 원본 URDF: 작업 폴더 `model/dual_piper.source.urdf`.
- 양 베이스 원점 Y=±0.0875m, RPY=[0,0,0]으로 모델링합니다.
- 뒤쪽 3030 기둥: 중심 [-0.0435,0,0.25]m, 단면 30mm, 높이 500mm.
- 양쪽 교체 그리퍼 최대 개방 100mm, D405 마운트 v6 포함.
- 거리 stop 1cm / resume 2cm. 현재 원형 볼록 모델 외부 여유 5mm.

## 15. 다음 채팅에서 해야 할 일과 하지 말아야 할 일

### 우선순위

1. 사용자가 원한 **외곽 충돌 검사 범위**가 현재 복원된 소스와 일치하는지 소스/설정부터 파악합니다.
   조립 내부 자기충돌 확대나 베이스/link2 보정으로 다시 방향을 틀지 마세요.
2. 실제 접촉을 놓친 원인을 모델 형상/좌표 대응, 검사 입력, 활성 쌍, 명령 전달, 정지 실행으로 나누어
   기존 로그와 소스로 추적합니다. 무엇이 확정이고 무엇이 가설인지 구분합니다.
3. 사용자에게 새 시뮬 명령을 안내할 경우 현재 소스의 전체 동작 성공은 미확인임을 밝힙니다.
   실패하면 해당 단계/검사 링크의 로그를 읽고 수정합니다. 허용폭/시간 제한/제외 쌍을 무작정 늘리지 마세요.
4. 더 필요한 실물 자료가 있으면 이미 촬영한 영상/정지 상태 자료처럼 이동을 요구하지 않는 자료부터 요청합니다.
   실물 재현을 자동 실행하지 마세요.
5. 일부 과거 문서에는 이미 철회한 자기충돌 확대/베이스 보정 설명이 남아 있습니다.
   특히 SIM_START_KO, CEREAL_TASK, SENSOR_MOUNT, INCIDENT, POSE_MISMATCH의 해당 설명을 현재 정책으로 오해하지 마세요.
   현재 문서와 실제 소스가 최신이며, 필요한 문서 정리는 후속 작업입니다.

### 금지/주의

- 실물 차단 해제, 드라이버 자동 활성화, 팔 이동, 자동 가드 리셋/재시도 금지.
- 사용자 검증 금지를 무시하고 테스트/빌드/새 시뮬을 자동 수행하지 마세요.
- 현재 접촉 사고를 단순 엔코더 허용폭으로 설명하지 마세요.
- 같은 팔 내부 검사 제외가 양팔 사이 접촉을 놓친 직접 원인이라고 주장하지 마세요.
- 원인 미확정인데 '해결 완료', '안전 검증 완료', '실물에서도 동일 동작'이라고 말하지 마세요.
- 과거 테스트/가상 피드백 통과 기록은 최신 구현의 결과가 아닙니다.
- 배포 ZIP/기존 build/install/현재 소스가 모두 같다고 가정하지 마세요.
- 불필요한 CAN 질문/임의 좌우 교환으로 다시 우선순위를 바꾸지 마세요.

## 새 채팅에 붙일 짧은 요청 예시

> 첨부한 CHAT_HANDOFF_2026-11-13_KO.md를 읽고 이어서 작업해.
> 실물 접촉을 놓친 원인과 시뮬–실물 자세 차이를 해결하는 게 우선이야.
> 같은 팔 조립 내부를 충돌로 잡는 변경은 철회된 상태고, 양팔 외곽/지그·환경 충돌을 봐야 해.
> CAN 변경은 보류하고, 실물 구동이나 자동 테스트·시뮬 실행은 하지 마.
> 현재 소스와 기존 로그를 확인해서 다음 작업을 진행해.
