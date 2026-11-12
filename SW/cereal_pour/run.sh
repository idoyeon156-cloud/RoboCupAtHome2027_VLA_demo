#!/usr/bin/env bash
set -e
variant_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$variant_root"
mode="${1:---help}"
if [[ "$mode" == --help ]]; then
  echo '사용: bash run.sh {distance_demo|distance_real|meshes_demo|meshes_real} [ROS launch 인자]'
  echo '설정: config/robot.yaml / 원본 모델: model/dual_piper.source.urdf'
  echo '모델 갱신: .venv/bin/python tools/prepare_model.py'
  echo '실물 기본값은 감시 모드. 차단 활성화: enforce:=true geometry_confirmed:=true'
  exit 0
fi
case "$mode" in
  distance_demo) package=piper_collision_guard; launch_file=collision_limit_visual_demo.launch.py ;;
  distance_real) package=piper_collision_guard; launch_file=guard.launch.py ;;
  meshes_demo) package=piper_moveit_guard; launch_file=moveit_box_demo.launch.py ;;
  meshes_real) package=piper_moveit_guard; launch_file=two_stage_guard.launch.py ;;
  *) echo "Unknown mode: $mode" >&2; exit 1 ;;
esac
shift
source /opt/ros/humble/setup.bash
if [[ -f "$variant_root/.venv/bin/activate" ]]; then
  source "$variant_root/.venv/bin/activate"
elif [[ -f "$variant_root/../../.venv/bin/activate" ]]; then
  source "$variant_root/../../.venv/bin/activate"
fi
python3 -c 'import pinocchio, numpy, scipy, yaml, rclpy' || {
  echo 'README.md의 프로젝트 가상환경 설치를 먼저 진행해 주세요.' >&2; exit 1;
}
# Refuse accidental box mode in the new polyhedron entry points.
for arg in "$@"; do
  case "$arg" in
    geometry:=boxes|editor:=true|collision_config:=*|config:=*|protection_path:=*|base_separation_m:=*|joint_center_limit_m:=*)
      echo '이 버전은 config/robot.yaml과 다면체 모델을 사용합니다. 설정 후 모델을 재생성하세요.' >&2; exit 1 ;;
  esac
done
python3 tools/prepare_model.py --check
# Explicit paths exclude the sibling legacy workspace, even when launched from another directory.
python3 /usr/bin/colcon build --symlink-install --base-paths \
  piper_dual_description piper_collision_guard piper_moveit_guard \
  --packages-select piper_dual_description piper_collision_guard piper_moveit_guard \
  --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source "$variant_root/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$variant_root/log/$mode}"
if [[ "$mode" == *_demo ]]; then
  export ROS_DOMAIN_ID="${COLLISION_DEMO_DOMAIN_ID:-178}"
  export ROS_LOCALHOST_ONLY=1
fi
# Real modes preserve networking/domain and do not start hardware drivers.
exec ros2 launch "$package" "$launch_file" "$@"
