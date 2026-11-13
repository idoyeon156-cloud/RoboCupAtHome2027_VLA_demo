#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$root/REAL_MOTION_HOLD_KO.md" ]]; then
  echo '드라이버 자동 활성화 차단: 손목 이상 동작 진단 중입니다. REAL_MOTION_HOLD_KO.md 참고.' >&2
  exit 1
fi
left_can="${1:-can0}"
right_can="${2:-can1}"
overlay_args=()
case "${3:-}" in
  '') ;;
  --arms-only) overlay_args+=(--disable-gripper) ;;
  *) echo '사용: bash run_piper_driver.sh can0 can1 [--arms-only]' >&2; exit 1 ;;
esac
if [[ "$left_can" == "$right_can" ]]; then
  echo '양팔에 같은 CAN 포트를 지정할 수 없습니다.' >&2
  exit 1
fi
for port in "$left_can" "$right_can"; do
  if ! ip -json link show dev "$port" | /usr/bin/python3 -c 'import json,sys; s=json.load(sys.stdin); sys.exit(0 if s and "UP" in s[0]["flags"] else 1)'; then
    echo "$port 가 UP 상태가 아닙니다. 연결과 포트 이름을 확인한 다음 실행하세요:" >&2
    echo "sudo ip link set $port up type can bitrate 1000000" >&2
    exit 1
  fi
done
source /opt/ros/humble/setup.bash
driver_ws="${PIPER_ROS_WS:-$HOME/piper_ros}"
if [[ ! -f "$driver_ws/install/setup.bash" ]]; then
  echo "드라이버 환경이 없습니다: $driver_ws/install/setup.bash" >&2
  exit 1
fi
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source "$driver_ws/install/setup.bash"
overlay="$root/build/driver_overlay"
/usr/bin/python3 "$root/tools/prepare_driver_overlay.py" --output "$overlay" "${overlay_args[@]}"
export PYTHONPATH="$overlay${PYTHONPATH:+:$PYTHONPATH}"
if [[ "${3:-}" == --arms-only ]]; then
  echo "ARMS ONLY: 왼팔=$left_can / 오른팔=$right_can. 관절 모터 활성화, 그리퍼 CAN 명령 전부 생략."
else
  echo "드라이버: 왼팔=$left_can / 오른팔=$right_can. 자동 활성화 시 그리퍼가 닫힙니다."
fi
exec ros2 launch piper start_two_piper.launch.py \
  can_left_port:="$left_can" can_right_port:="$right_can" \
  auto_enable:=true log_level:=warn
