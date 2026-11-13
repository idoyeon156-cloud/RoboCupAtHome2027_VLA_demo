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
export CEREAL_RUN_ID="${CEREAL_RUN_ID:-$(date +%Y-%m-%d_%H-%M-%S)_$$}"
if [[ "$mode" == *_demo ]]; then
  export ROS_DOMAIN_ID="${COLLISION_DEMO_DOMAIN_ID:-178}"
  export ROS_LOCALHOST_ONLY=1
fi
export CEREAL_PACKAGE_ROOT="$variant_root"
# Planning requires the newly built native MoveIt executable. No hardware is launched.
# Guarded cereal simulation accepts MoveIt plans only.
# Hardware replay must use the completed immutable simulation snapshot.
if [[ "$mode" == meshes_demo || "$mode" == meshes_real ]]; then
  cereal_requested=false
  custom_sequence=false
  arms_only=false
  planner_args=()
  task_config="$variant_root/config/cereal_task.yaml"
  launch_args=()
  for arg in "$@"; do
    case "$arg" in
      arms_only:=true) arms_only=true; planner_args+=(--arms-only); launch_args+=("$arg") ;;
      arms_only:=false) launch_args+=("$arg") ;;
      cereal:=true) cereal_requested=true; if [[ "$mode" == meshes_real ]]; then launch_args+=("$arg"); fi ;;
      sequence_path:=*) custom_sequence=true; launch_args+=("$arg") ;;
      task_config:=*) task_config="${arg#task_config:=}" ;;
      *) launch_args+=("$arg") ;;
    esac
  done
  set -- "${launch_args[@]}"
  if [[ "$cereal_requested" == true ]]; then
    if [[ "$mode" == meshes_demo ]]; then
      launch_file=cereal_guarded_rviz.launch.py
    elif [[ -f "$variant_root/REAL_MOTION_HOLD_KO.md" ]]; then
      echo '실물 실행 차단 중: REAL_MOTION_HOLD_KO.md 참고' >&2
      exit 1
    fi
    if [[ "$custom_sequence" == false ]]; then
      if [[ "$mode" == meshes_demo ]]; then
        sequence="$variant_root/model/cereal_task_sequence.yaml"
        if [[ "$arms_only" == true ]]; then sequence="$variant_root/model/cereal_arms_only_sequence.yaml"; fi
        if ! python3 tools/plan_cereal_task.py --config "$task_config" --output "$sequence" "${planner_args[@]}" --check; then
          echo 'MoveIt으로 전체 경로를 계획합니다. 실패하면 RViz 동작을 시작하지 않습니다.'
          python3 tools/plan_cereal_task.py --config "$task_config" --output "$sequence" "${planner_args[@]}"
        fi
      else
        sequence="$(python3 -c 'from piper_collision_guard.plan_snapshot import completed_sequence; print(completed_sequence())')"
      fi
      set -- "$@" "sequence_path:=$sequence"
    fi
  fi
fi
# Real modes preserve networking/domain and do not start hardware drivers.
exec ros2 launch "$package" "$launch_file" "$@"
