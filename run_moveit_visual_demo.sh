#!/usr/bin/env bash
# MoveIt 충돌 박스 시각화. 실제 로봇 드라이버를 실행하지 않습니다.
set -e
if [[ "${1:-}" == --help ]]; then
  echo '사용: bash run_moveit_visual_demo.sh [gui:=false] [rviz:=false]'
  echo '기본: 병렬 관절 조절기 + RViz, 박스/충돌 지점/겹침 깊이 표시.'
  exit 0
fi
MOVEIT_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MOVEIT_MODE=demo
source "$MOVEIT_REPO_ROOT/tools/moveit_runtime.sh"
export ROS_DOMAIN_ID="${COLLISION_DEMO_DOMAIN_ID:-179}"
export ROS_LOCALHOST_ONLY=1
exec ros2 launch piper_moveit_guard moveit_box_demo.launch.py "$@"
