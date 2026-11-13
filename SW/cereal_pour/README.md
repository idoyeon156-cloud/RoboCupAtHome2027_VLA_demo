# 양팔 PiPER 실물 배포와 RViz 시뮬레이터

2026-11-13 기준, **그리퍼 명령을 제외한 팔 전용 모드에서 전체 시퀀스와 제로 복귀를 확인했습니다.**
동작 속도와 구간 끊김·지터링을 개선한 현재 소스를 배포합니다. 잔여 떨림과 일부 코너의 정지는
여전히 조사 중입니다. 그리퍼는 SDK 직접 제어에서도 활성화되지 않아 제외 모드를 유지합니다.
구동 확인은 물체 없는 팔 경로에 한하며, 실제 파지·시리얼 붓기나 충돌·제동 성능 검증 완료를 뜻하지 않습니다.

## 현재 배포 정책과 향후 계획

**안전상의 이유로, 현재 실물 배포본은 먼저 같은 작업 폴더에서 시뮬레이션을 끝까지 완료해야
실제 구동이 가능하도록 구성했습니다. 향후 동작이 안정화되면 시뮬레이션 없이 실물을 실행할 수
있도록 변경할 예정입니다. 현재는 이 선행 조건을 유지합니다.**

이는 MoveIt 자체의 필수 조건이 아니라 패키지의 실행 정책입니다. 완료한 시뮬의 경로·소스·모델·
설정·속도가 실물과 같아야 합니다. 변경이 없으면 완료 기록을 재사용하므로 매번 시뮬을 돌릴 필요는 없습니다.
중간 종료, 다른 작업 폴더/기기로 이동, 소스·설정 변경 시 다시 완료해야 합니다.
배포 ZIP에는 로컬 시뮬 완료 인증 기록을 포함하지 않습니다.

## 설치와 현재 확인된 실행

Ubuntu 22.04 / ROS 2 Humble 기준 의존성은 [SETUP.md](SETUP.md)를 참고하세요.
모든 명령은 이 README가 있는 폴더에서 실행합니다.

먼저 CAN/실물 드라이버 없이 팔 전용 시뮬을 실행합니다.

```bash
bash run_cereal_rviz.sh arms_only:=true
```

MoveIt OMPL RRTConnect/Bullet이 경로를 계획합니다. 거리 가드 → MoveIt 메시 가드 → 가상 드라이버의
피드백을 RViz에서 표시하며 두 가드는 enforce입니다. `SIMULATION COMPLETE` 후 종료합니다.
계획 실패나 중간 종료는 완료가 아닙니다. 관성·접촉력·오버슈트를 재현하는 물리 시뮬은 아닙니다.

물체 없이 양팔 관절 영점·그리퍼 닫힘 상태에서 각각 별도 터미널로 실행합니다.
SDK/기존 드라이버와 중복 실행하지 않습니다. 외부 `~/piper_ros` 설치가 필요하며,
위치가 다르면 `PIPER_ROS_WS` 환경 변수로 지정합니다.

```bash
# 터미널 1: 관절 모터 활성화, 그리퍼 CAN 명령은 모두 생략
bash run_piper_driver.sh can0 can1 --arms-only
```

```bash
# 터미널 2: 완료한 팔 전용 경로와 두 enforce 가드
bash run_cereal_real.sh enforce:=true geometry_confirmed:=true arms_only:=true
```

```bash
# 터미널 3: 읽기 검사에서 PASS 후 전체 시퀀스를 한 번 실행
bash run_real_control.sh check
bash run_real_control.sh run
```

제로 → 알파 → 가상 그릇·시리얼 파지/붓기 경로 → 알파 경유 → 제로를 연속 실행합니다.
알파에서 추가 start는 필요 없습니다. `Finished; holding return_home`가 전체 완료입니다.
실패 시 자동 리셋·재시도하지 않습니다. `geometry_confirmed:=true`는 사용자 확인 플래그이며
실물 형상을 자동 측정하거나 검증하지 않습니다.

```bash
# 소프트웨어 목표 전송 정지 요청 (하드웨어 비상정지가 아님)
bash run_real_control.sh stop
```

## CAN 좌우 기준

| 논리 팔 | 기본 CAN | 역할 | 로봇 뒤에서 그리퍼 방향 | 그리퍼 쪽에서 베이스 방향 |
| --- | --- | --- | --- | --- |
| left (파랑) | can0 | 그릇 팔 | 왼쪽 | 오른쪽 |
| right (주황) | can1 | 시리얼 팔 | 오른쪽 | 왼쪽 |

패키지의 전제이며 물리 위치를 자동 판별하지 않습니다. 재연결 후 실제 케이블 대응을 확인합니다.

## 구성과 개선 범위

- 목표 IK: Pinocchio FK + SciPy, 관절 경로: native MoveIt OMPL/Bullet.
- 실행 중에는 고정 경로를 보간하고 충돌을 검사합니다. 계속 재계획하는 구조가 아닙니다.
- 메시에는 교체 그리퍼·양쪽 D405 마운트·고정 기둥을 포함합니다.
- 같은 팔 조립 내부 쌍은 제외하고 양팔 사이와 로봇–고정 환경을 검사합니다.
- 불필요한 대기 제거, 5차 시간 보간, 추종 오차에 따른 점진적 감속, 복귀 알파 감속을 적용했습니다.
- 거의 일직선인 경유점은 MoveIt 재검사에 통과한 경우에만 연결하며, 실제 코너에서는 정지합니다.
- 실물은 자체 위치 목표 실행기와 PiPER MOVE J를 사용합니다. FollowJointTrajectory 직접 실행은 아닙니다.
- ROS/CAN 없는 수치·제어 흐름·그리퍼 차단 테스트 12개 통과. 사용자 시뮬 완료 및 실물 제로 복귀 확인.
  잔여 떨림의 직접 원인은 미확정입니다.

물체 설정은 `config/cereal_task.yaml`, 설치/형상은 `config/robot.yaml`과
`model/dual_piper.source.urdf`입니다. 모델 수정 후 `tools/prepare_model.py`로 재생성합니다.

- [팔 전용 모드·실행 개선 상세](ARMS_ONLY_KO.md)
- [실물 구동 결과 기록](REAL_RUN_OBSERVATIONS_2026-11-13_KO.md)
- [RViz 구성 상세](SIM_START_KO.md)
- [D405 장착 형상](SENSOR_MOUNT.md), [고정 기둥](REAR_PROFILE.md)
- [배포 ZIP](../bundles/README.md)

과거 사고 조사·인계 문서는 당시 상태의 기록으로 보존합니다. 과거 전면 실행 차단·미검증 표기보다
현재 실행 안내는 이 README와 ARMS_ONLY_KO.md를 우선합니다. 별도로 `REAL_MOTION_HOLD_KO.md`가
존재하는 설치본은 추가 임시 차단을 적용합니다.
