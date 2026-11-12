#!/usr/bin/env bash
# 실물 연결용 프로토타입. 기본 감시 모드이며 드라이버를 실행하지 않습니다.
set -e
if [[ "${1:-}" == --help ]]; then
  echo '사용: bash run_moveit_real_guard.sh [enforce:=true geometry_confirmed:=true]'
  echo '기본: Pinocchio + MoveIt 감시. Piper 드라이버/텔레옵은 별도 실행.'
  echo '명령 전달 설정은 실제 형상 및 정지 동작 검증 후 사용하세요.'
  echo '기본 형상: 전체 다면체. 이전 박스 편집: geometry:=boxes editor:=true (시뮬용).'
  exit 0
fi
MOVEIT_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MOVEIT_MODE=real
source "$MOVEIT_REPO_ROOT/tools/moveit_runtime.sh"
# Real robot: preserve the user's ROS domain and networking configuration.
exec ros2 launch piper_moveit_guard two_stage_guard.launch.py "$@"
