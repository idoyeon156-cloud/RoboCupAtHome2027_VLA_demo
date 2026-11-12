#!/usr/bin/env bash
# 충돌 제한 시각화 데모 전용. 실제 Piper 드라이버를 실행하지 않습니다.
set -e
DEMO_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$DEMO_REPO_ROOT"
if [[ ! -f .venv/bin/activate || ! -f install/setup.bash ]]; then
  echo "먼저 piper_collision_guard/VISUAL_DEMO.md의 설치·빌드를 진행해 주세요."
  exit 1
fi
source /opt/ros/humble/setup.bash
source .venv/bin/activate
# colcon invokes this interpreter without quoting; use the system path for its stdlib-only setup helper.
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
export ROS_DOMAIN_ID="${COLLISION_DEMO_DOMAIN_ID:-178}"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$DEMO_REPO_ROOT/log/visual_demo}"
exec ros2 launch piper_collision_guard collision_limit_visual_demo.launch.py "$@"
