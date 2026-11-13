#!/usr/bin/env bash
set -e
variant_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$variant_root/REAL_MOTION_HOLD_KO.md" ]]; then
  echo '실물 자동 실행 차단: 손목 이상 동작의 원인과 경로를 정리해야 합니다. REAL_MOTION_HOLD_KO.md 참고.' >&2
  exit 1
fi
exec bash "$variant_root/run.sh" meshes_real cereal:=true joint_speed_scale:=1.5 driver_speed_ratio:=20 "$@"
