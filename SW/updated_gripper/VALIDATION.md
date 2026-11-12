# 새 그리퍼 안전장치 검증 — 2026-11-12

## 모델

- 독립 piper_dual_updated_gripper_description 패키지의 URDF와 ROS 패키지 URI 이름을 제외하고 정확히 일치. STL 11개도 동일.
- 양팔 베이스 간격 175 mm, 총 개방량 100 mm, 플랜지와 새 그리퍼 베이스 포함.
- prepare_model.py --check의 입력·생성물 해시 검사 통과. MoveIt 양팔 64개 볼록 다면체.
- 거리 방식의 기존 7개 보호 구, 10 mm 정지 / 40 mm 재개 여유와 래치 정책 유지.

## 실제 실행한 검사

Ubuntu 22.04 / ROS 2 Humble, 작업 폴더 전용 Pinocchio 2.7.0 가상환경에서 실행했습니다.

- ROS 패키지 3개 colcon 빌드 성공.
- 설정 변경, 거리 계산, 조절기, 메시 형상: 19개 통과.
- 교체 URDF·STL 일치, 플랜지 충돌 포함, 100 mm 범위, 메시 보호 구, preview, 박스 형상, 튜닝: 25개 통과.
- 분리된 로컬 ROS 도메인 181에서 네 실행 경로의 상태 수신, 기본 감시 모드, 드라이버 명령 미발행 및 변경 폭·카메라 충돌 검사: 5개 통과.
- 로컬 도메인 177에서 거리 감시·명령 차단 검사: 1개 통과.
- 로컬 도메인 181에서 MoveIt 메시 충돌 서비스·차단·래치·피드백 소실 및 1차→2차 전달 검사: 2개 통과.

합계 52개 검사 통과. 시뮬 ROS 검증은 gui:=false rviz:=false로 수행했습니다. 실제 RViz 화면과 실물 로봇 동작은 검증하지 않았으며 실제 드라이버는 연결하지 않았습니다.

## 재현

README.md의 설치 절차와 별도 셸 환경을 사용합니다. ROS 로그는 작업 폴더 아래로 지정합니다.

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/log/verification"
python tools/prepare_model.py --check
python -m pytest -q test/test_variant.py test/test_updated_gripper.py \
  piper_collision_guard/test/test_distance.py piper_collision_guard/test/test_degree_gui.py \
  piper_collision_guard/test/test_mesh_coverage.py piper_collision_guard/test/test_preview.py \
  piper_moveit_guard/test/test_mesh_geometry.py piper_moveit_guard/test/test_geometry.py \
  piper_moveit_guard/test/test_tuning.py
ROS_DOMAIN_ID=181 RUN_VARIANT_ROS_TESTS=1 python -m pytest -q test/test_variant_ros.py
ROS_DOMAIN_ID=177 RUN_ROS_GUARD_TESTS=1 python -m pytest -q piper_collision_guard/test/test_ros_guard.py
ROS_DOMAIN_ID=181 RUN_MOVEIT_GUARD_TESTS=1 python -m pytest -q \
  'piper_moveit_guard/test/test_ros_moveit.py::test_moveit_collision_service_and_command_interlock[meshes]' \
  'piper_moveit_guard/test/test_ros_moveit.py::test_two_stage_launch_routes_only_through_moveit[meshes]'
```
