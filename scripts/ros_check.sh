#!/usr/bin/env bash
# Usage: scripts/ros_check.sh <export.zip>
# Generates the ROS 2 workspace from a cad-to-urdf export and checks it in a ROS 2 Jazzy container:
# build, controllers active, TF, MoveIt plan+execute, EAIK and TRAC-IK.
# For an EAIK-solvable test arm: tests/ros_helpers.py six_axis_zip (see backend/tests).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
ZIP=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
WS=$(mktemp -d)
trap 'rm -rf "$WS"' EXIT
PY=${PYTHON:-$HOME/miniforge3/envs/cad-to-urdf/python.exe}
host() { if command -v cygpath >/dev/null; then cygpath -w "$1"; else echo "$1"; fi; }

(cd "$ROOT/backend" && OPENBLAS_NUM_THREADS=1 "$PY" -m app.ros.cli "$(host "$ZIP")" -o "$(host "$WS")")
docker build -q -t cad-to-urdf-jazzy "$(host "$ROOT/docker/jazzy")" >/dev/null
MSYS_NO_PATHCONV=1 docker run --rm \
  -v "$(host "$WS"):/ws" -v "$(host "$ROOT/docker/jazzy/check"):/check:ro" \
  cad-to-urdf-jazzy bash /check/run.sh
