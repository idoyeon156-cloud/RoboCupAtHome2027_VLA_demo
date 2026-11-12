# 새 그리퍼 URDF용 PiPER 안전장치

교체한 그리퍼 베이스와 플랜지를 적용한 URDF로 기존 두 안전장치를 구성했습니다. 교체 전 버전은 [../original_gripper/](../original_gripper/README.md), 씨리얼 시퀀스는 [../cereal_pour/](../cereal_pour/README.md)에 있습니다.

이 폴더에서 기존과 같은 명령어를 사용합니다. J1~J6 슬라이더와 그리퍼 개방 조절로 안전장치를 확인할 수 있습니다.

| 안전장치 | RViz 시뮬레이터 | 실물용 실행 |
| --- | --- | --- |
| 조인트 간 거리 | `bash run_distance_rviz.sh` | `bash run_distance_real.sh` |
| MoveIt 메시 | `bash run_meshes_rviz.sh` | `bash run_meshes_real.sh` |

`run_meshes_real.sh`는 기존과 동일하게 1차 거리 검사 → 2차 MoveIt 검사로 연결합니다. 실물 기본값은 감시 모드이며 드라이버는 별도로 실행합니다.

모델: [표시·거리 검사 URDF](piper_dual_description/urdf/dual_piper.urdf), [MoveIt URDF](piper_moveit_guard/config/dual_piper_meshes.urdf), [설정](config/robot.yaml). 독립 [새 그리퍼 URDF 패키지](../../HW/URDF/updated_gripper)와 같은 형상·장착 위치·관절 한계를 사용합니다. 베이스 간격 175 mm, 총 개방량 100 mm, J7 0~50 mm / J8 −50~0 mm입니다.

거리 방식은 기존처럼 관절 원점 6개와 그리퍼 기준점에 설정된 보호 구를 사용합니다. MoveIt 방식은 새 베이스와 플랜지를 포함한 양팔 64개 볼록 다면체 메시로 검사합니다.

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

원본 `collision/`에서 만든 build/install은 이 버전에서 사용하지 않습니다. 실행기는 `SW/updated_gripper/` 안에 별도로 빌드합니다.

## 바뀌었을 때 무엇을 수정하나

아래 경로는 `SW/updated_gripper/` 폴더 기준입니다. 설정값과 원본 모델을 수정한 뒤 아래의 공통 절차로 재생성합니다.

| 바뀐 부분 | 수정할 파일 | 수정할 항목 |
| --- | --- | --- |
| 양팔 간격·높이·위치 | config/robot.yaml | mounts → left / right → xyz 값 변경. 순서: [x, y, z] |
| 팔 설치 각도 | config/robot.yaml | mounts → left / right → rpy 값 변경 |
| 그리퍼 최대 개방 폭 | config/robot.yaml | grippers → left / right → max_opening_m 값 변경 |
| 그리퍼 외형·손가락 길이·두께 | piper_dual_description/meshes/ + model/dual_piper.source.urdf | 해당 STL 교체, 해당 링크의 visual / collision 메시 경로·장착 위치 수정 |
| 센서·카메라·마운트 추가 | config/robot.yaml | attachments에 장착 링크(parent), 위치(xyz), 각도(rpy), 형상(geometry) 추가 |
| 다면체 분할·충돌 여유 | config/robot.yaml | polyhedra → sections의 axis / cuts_m, margin_m 수정 |
| 1차 보호 구 크기·중심 | config/robot.yaml | joint_distance → protection → left / right의 radii_m / offsets_m 수정 |
| 1차 정지·재개 간격 | config/robot.yaml | joint_distance → stop_clearance_m / resume_clearance_m 수정 |

- 시뮬 조절기: 각도 °(디그리), 손가락 이동 mm. 내부 계산용 rad·m로 자동 변환됩니다.
- 설정 파일: 거리 m, 설치 각도 rpy는 rad. 예: 90° = 약 1.5708rad.
- 센서·마운트는 config/attachments.example.yaml을 참고해 robot.yaml의 attachments에 복사합니다. 예시 파일만 수정하면 적용되지 않습니다.
- 부품 외형을 바꾸면 1차 보호 구도 확인합니다. 추가 부품의 부피는 2차 다면체에서 검사합니다.
- 조립상 맞닿는 링크는 해당 attachment의 touch_links 또는 allowed_collision_pairs로 지정합니다.

`max_opening_m`은 기존 드라이버와 같은 **양쪽 손가락의 총 이동량**입니다. joint7에 +opening/2, joint8에 -opening/2를 적용합니다. 실제 손가락 안쪽 면 간 거리와 다를 수 있으므로 새 그리퍼에서는 닫힘 기준과 드라이버 피드백 의미를 확인하세요.
최대 폭을 바꾸면 생성 URDF의 관절 범위, 1차·2차 입력 허용 범위, 슬라이더 범위가 함께 바뀝니다. 장착 원점이나 손가락 메시 자체의 크기는 자동으로 변하지 않습니다.

기본은 좌우 각각 100mm입니다. 레일 몸체 폭은 공식 STL에서 약 164.08mm입니다. 1차의 과거 80mm 고정 검사와 2차의 70mm 고정 검사를 제거하고 **실제로 로드한 URDF**의 범위를 읽도록 변경했습니다.

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

## 나중에 팔 아래에 리프트를 달면

| 사용 방식 | 현재 반영 방법 |
| --- | --- |
| 높이를 조절한 뒤 멈춘 상태에서 팔 사용 | 현재 지원. config/robot.yaml의 mounts → 해당 팔 → xyz의 z를 실제 팔 베이스 높이로 수정하고 모델 재생성·재실행 |
| 운전 중 리프트 높이가 계속 변함 | 현재 미지원. 리프트 관절 모델, 실제 높이 피드백, 이동 구간 검사를 추가해야 함 |

설정으로 반영하는 방식은 고정한 높이에서 팔을 검사하는 것이며, 높이를 바꾸는 동안의 리프트 이동 자체는 검사하지 않습니다.

예를 들어 오른팔 베이스를 world 기준 20cm 높이에 고정했다면 mounts.right.xyz를 `[0.0, -0.0875, 0.20]`으로 설정합니다. z는 이동량이 아니라 **공통 world 기준 팔 베이스의 실제 높이**입니다. 양팔 높이는 각각 설정할 수 있습니다.

팔 베이스 높이 설정만으로 리프트 기둥·플랫폼의 충돌 형상이 생기지는 않습니다. 고정된 높이에서 사용하는 구성이라면 해당 구조물을 attachments 또는 원본 URDF의 고정 링크로 추가하고 조립상 접촉하는 링크만 검사 제외로 지정합니다.

운전 중 움직이는 리프트를 지원할 때는 기존 PiPER 팔 모델과 다면체 검사 계산을 재사용할 수 있습니다. 다만 현재 모델 생성기는 PiPER의 기존 관절 구성만 허용하고 베이스를 고정 조인트로 처리하므로, 추가 축 허용·상태 입력을 확장해야 합니다. 1차와 2차에 같은 실제 높이를 반영하고, 피드백 누락 처리와 리프트 이동 중 충돌 검사·정지 연동도 추가해야 합니다. 설정의 z만 바꿔서는 움직이는 리프트에 대응할 수 없습니다.

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
