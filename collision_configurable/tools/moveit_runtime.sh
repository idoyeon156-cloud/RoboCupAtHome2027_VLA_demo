#!/usr/bin/env bash
# Shared build/environment setup for the MoveIt demo and robot guard launchers.
# Sourced by a launcher after MOVEIT_REPO_ROOT and MOVEIT_MODE are set.
set -e
cd "$MOVEIT_REPO_ROOT"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS2 Humble이 필요합니다. README_START.md 또는 piper_moveit_guard/README.md를 확인하세요.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
moveit_python=/usr/bin/python3
if [[ -n "${VIRTUAL_ENV:-}" && -x "$VIRTUAL_ENV/bin/python" ]]; then
  moveit_python="$VIRTUAL_ENV/bin/python"
elif [[ -x "$MOVEIT_REPO_ROOT/.venv/bin/python" ]]; then
  source "$MOVEIT_REPO_ROOT/.venv/bin/activate"
  moveit_python="$VIRTUAL_ENV/bin/python"
elif [[ -x "$HOME/piper_vla_env/bin/python" ]]; then
  source "$HOME/piper_vla_env/bin/activate"
  moveit_python="$VIRTUAL_ENV/bin/python"
fi
if [[ "$MOVEIT_MODE" == real ]]; then
  "$moveit_python" -c 'import pinocchio, rclpy' || {
    echo 'Pinocchio를 설치한 가상환경을 먼저 활성화하세요. README_START.md 참고.' >&2
    exit 1
  }
fi
"$moveit_python" /usr/bin/colcon build --symlink-install --packages-select \
  piper_dual_description piper_collision_guard piper_moveit_guard
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
