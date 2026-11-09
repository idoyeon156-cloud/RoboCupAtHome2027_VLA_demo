# 수정해서 재사용하는 PiPER 충돌 방지 버전

기존 `../collision/`을 보존하고 별도로 만든 작업 공간입니다. 기준 소스는 저장소 커밋 `d14edba798b733a6f4891cc278b465386230cd85`입니다.
PiPER 6축, 두 손가락의 대칭 이동과 드라이버 메시지 규약은 유지하며, 배치·최대 개방 폭·부품 형상을 변경할 수 있습니다.
2차 기본 형상은 기존과 같은 구간별 볼록 다면체입니다. 기본 설정에서는 양팔 합계 62개와 기존 5mm 지지 평면 여유를 유지합니다.

## 네 가지 실행

이 폴더에서 실행합니다. 아래 실행기는 모델 최신 여부를 확인하고 이 폴더의 세 패키지만 빌드한 후 실행합니다.

| 용도 | 실행 |
|---|---|
| 1차 조인트 거리 + RViz 시뮬 | `bash run_distance_rviz.sh` |
| 1차 조인트 거리 실물용 | `bash run_distance_real.sh` |
| 2차 다면체 메시 + RViz 시뮬 | `bash run_meshes_rviz.sh` |
| 실물용 1차 → 2차 다면체 직렬 구성 | `bash run_meshes_real.sh` |

실물용 기본값은 **감시 모드**입니다. 드라이버와 텔레옵은 별도 실행하며, 실제 형상과 정지 동작을 확인한 뒤 명령 전달을 활성화합니다.

```bash
bash run_distance_real.sh enforce:=true geometry_confirmed:=true
# 또는 통합 모드. 위 단독 모드와 동시에 실행하지 않습니다.
bash run_meshes_real.sh enforce:=true geometry_confirmed:=true
```

통합 모드 경로: `/teleop/{left,right}_target` → 1차 → `/pinocchio_checked/{left,right}_target` → 2차 → `/joint_ctrl_cmd_{left,right}`.
피드백은 `/joint_states_left`, `/joint_states_right`를 사용합니다. 기존 타임스탬프 검사·래치·리셋을 유지합니다.
리셋 서비스는 `/collision_guard/reset`, `/moveit_guard/reset`입니다.

시뮬에는 드라이버 출력이 없고, 별도 로컬 ROS 도메인 178을 사용합니다. `gui:=false rviz:=false`로 화면 없이 확인할 수 있습니다.
실물 실행기는 사용자의 ROS 도메인과 네트워크 설정을 유지합니다. 원본 버전과 ROS 패키지·토픽 이름이 같으므로 같은 셸에서 두 workspace를 섞거나 두 실물 guard를 동시에 실행하지 마세요.

## 처음 설치

ROS 2 Humble과 MoveIt/Bullet, RViz, joint_state_publisher, python_qt_binding이 필요합니다.
기존 환경을 쓰거나 아래처럼 프로젝트 전용 환경을 만듭니다.

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python tools/prepare_model.py
bash run_distance_rviz.sh
```

원본 `collision/`에서 만든 build/install은 이 버전에서 사용하지 않습니다. 실행기는 `collision_configurable/` 안에 별도로 빌드합니다.

## 바뀌었을 때 무엇을 수정하나

| 변경 | 수정 위치 |
|---|---|
| 양팔 간격·설치 높이·기울기 | `config/robot.yaml`의 `mounts.left/right.xyz/rpy` |
| 그리퍼 최대 개방 폭 | 같은 파일의 `grippers.left/right.max_opening_m` |
| 그리퍼 메시·손가락 길이·두께·장착 형상 | `model/dual_piper.source.urdf`와 `piper_dual_description/meshes/`의 해당 메시 |
| 센서·마운트 추가 | `config/robot.yaml`의 `attachments` 또는 원본 URDF에 고정 링크 추가 |
| 특정 부품의 다면체 분할 조정 | `polyhedra.sections`의 링크 이름 또는 STL stem 항목 |
| 다면체 생성 여유 | `polyhedra.margin_m` |
| 1차 보호 구 위치·크기 | `joint_distance.protection`의 좌우 7개 `offsets_m`, `radii_m` |
| 1차 정지·재개 간격 | `joint_distance.stop_clearance_m/resume_clearance_m` |
| 장착 접촉을 허용할 링크 쌍 | attachment의 `touch_links` 또는 `allowed_collision_pairs` |

단위: 위치·거리 m, 회전 rpy rad. `max_opening_m`은 기존 드라이버와 같은 **양쪽 손가락의 총 이동량**입니다. joint7에 +opening/2, joint8에 -opening/2를 적용합니다. 실제 손가락 안쪽 면 간 거리와 다를 수 있으므로 새 그리퍼에서는 닫힘 기준과 드라이버 피드백 의미를 확인하세요.
최대 폭을 바꾸면 생성 URDF의 관절 범위, 1차·2차 입력 허용 범위, 슬라이더 범위가 함께 바뀝니다. 장착 원점이나 손가락 메시 자체의 크기는 자동으로 변하지 않습니다.

기본은 좌우 각각 70mm입니다. 1차의 과거 80mm 고정 검사와 2차의 70mm 고정 검사를 제거하고 **실제로 로드한 URDF**의 범위를 읽도록 변경했습니다.

## 수정 후 공통 절차

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
# config/robot.yaml 또는 model/dual_piper.source.urdf / 해당 메시 수정
python tools/prepare_model.py
bash run_distance_rviz.sh
bash run_meshes_rviz.sh
```

두 시뮬은 하나씩 실행·종료하세요. 생성 도구는 다음을 함께 갱신합니다.

- `piper_dual_description/urdf/dual_piper.urdf`: 설치 배치와 개방 범위를 반영한 1차/표시용 모델
- `piper_moveit_guard/config/dual_piper_meshes.urdf`: 같은 배치·관절에 다면체 충돌 형상을 적용한 2차 모델
- `piper_moveit_guard/meshes/collision/`: 다면체 메시
- `piper_moveit_guard/config/dual_piper.srdf`: 기본 검사 제외와 추가 부품 접촉 정책
- `piper_collision_guard/config/protection.yaml`, `guard.yaml`: 1차 설정
- `model/manifest.json`: 입력과 출력의 해시

**위 생성 파일은 직접 편집하지 않습니다.** 실행 전에 해시로 변경 누락을 검사하며, 원본이나 설정이 바뀌었는데 재생성하지 않았으면 실행을 중단합니다.
설정의 잘못된 분할 위치나 누락 메시 등은 출력 교체 전 검사합니다. 생성 도중 중단돼 출력이 섞여도 manifest 검사가 걸리므로 다시 생성하면 됩니다.

## 센서·마운트 예시

`config/attachments.example.yaml`의 항목을 `config/robot.yaml`의 `attachments: []`에 옮기고 위치·크기를 수정합니다.
기본으로는 장착물을 추가하지 않았습니다. box, cylinder, sphere, STL mesh를 지원합니다. sphere는 보수적인 외접 박스 기반 다면체, cylinder는 외접 다각기둥으로 변환합니다. 더 정밀한 형상은 STL을 지정하세요.

메시 부품은 `package://piper_dual_description/meshes/부품.STL`로 참조하는 것이 배포하기 쉽습니다. ASCII/바이너리 STL, 충돌 origin/rpy, 양의 mesh scale, 링크당 여러 collision을 지원합니다.
분할 설정이 없는 새 메시는 하나의 볼록 형상으로 감쌉니다. 오목한 부품에서 과하게 감싸면 링크 이름을 키로 분할을 추가하세요. 현재 한 링크에 최대 8개 다면체를 허용합니다.

현재 팔 본체 내부의 자기 충돌 제외 정책은 유지합니다. **추가 장착물은 자동으로 같은 팔 제외 목록에 넣지 않습니다.** 기본적으로 다른 충돌 형상과 검사하며, 실제로 맞닿아야 하는 장착 링크만 `touch_links`로 명시합니다. 좌우 팔 사이 검사는 강제로 활성화됩니다.

1차는 기존의 **관절 원점 6개와 그리퍼 중심의 보호 구 거리 검사**입니다. 부품 메시를 바꿨다고 보호 구가 자동으로 확대되지는 않습니다. 필요하면 보호 구 위치·반경을 조정하세요. 추가 센서·마운트의 부피는 2차에서 검사합니다. 1차와 2차는 같은 설치 배치와 관절 한계를 사용하지만 역할은 다릅니다.

## 검증

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
python -m pytest -q test/test_variant.py piper_collision_guard/test/test_distance.py \
  piper_collision_guard/test/test_degree_gui.py piper_collision_guard/test/test_preview.py
# 실제 드라이버 없이, 격리된 ROS 도메인에서 실행하는 통합 검사
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 RUN_VARIANT_ROS_TESTS=1 \
  python -m pytest -q test/test_variant_ros.py
```

출고/장착 형상이 바뀌면 닫힘·중간·최대 개방, 새 부품 근접 자세, 높이·간격 변경을 시각적으로 확인하고 실물 단계에서 검증하세요. 리프트 운전 중 높이 갱신이나 다른 관절 구조는 이번 구현 범위에 포함하지 않았습니다.
