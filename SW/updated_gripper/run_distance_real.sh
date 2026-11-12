#!/usr/bin/env bash
set -e
variant_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$variant_root/run.sh" distance_real "$@"
