#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$root/.venv/bin/activate" ]]; then
  source "$root/.venv/bin/activate"
elif [[ -f "$root/../../.venv/bin/activate" ]]; then
  source "$root/../../.venv/bin/activate"
fi
if [[ ! -f "$root/install/setup.bash" ]]; then
  echo '먼저 다른 터미널에서 bash run_cereal_real.sh enforce:=true geometry_confirmed:=true 를 실행하세요.' >&2
  exit 1
fi
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source "$root/install/setup.bash"
exec python3 "$root/tools/real_control.py" "$@"
