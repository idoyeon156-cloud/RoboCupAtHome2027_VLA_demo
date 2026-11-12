#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$root/run_meshes_rviz.sh" cereal:=true "$@"
