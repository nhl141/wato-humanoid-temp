#!/usr/bin/env bash
# Run a GL40 bench move inside the interfacing container, then plot it on the host.
#
# Telemetry is written by the script itself (always on) into the repo's outputs/ directory,
# which is bind-mounted into the container. Plotting needs matplotlib/numpy, which deliberately
# are NOT in the robot-control image, so this wrapper does that half on the host via `uv`.
#
#   tools/gl40_move.sh --id 22 --deg 40 --kp 1.22 --max-track-err 12
#   tools/gl40_move.sh --id 22 --monitor              # Ctrl-C to stop
#   GL40_TOOL=gl40_bench.sh tools/gl40_move.sh --id 22 --step 5 --sweep "0.61,1.22"
#
# Env: GL40_SERVICE (default interfacing), GL40_TOOL (gl40_mit_move.py | gl40_bench.py),
#      GL40_NO_PLOT=1 to skip plotting.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE="${GL40_SERVICE:-interfacing}"
TOOL="${GL40_TOOL:-gl40_mit_move.py}"
CONTAINER="$(docker ps --filter "name=-${SERVICE}-" --format '{{.Names}}' | head -1)"

if [[ -z "$CONTAINER" ]]; then
  echo "No running '${SERVICE}' container found. Start it with:  ./watod up -d" >&2
  exit 1
fi

mkdir -p "${REPO}/outputs/gl40_bench"
BEFORE="$(ls -1 "${REPO}/outputs/gl40_bench" 2>/dev/null | wc -l)"

echo "=> ${CONTAINER}: ${TOOL} $*"
# -t so Ctrl-C reaches the script (its handler frees the motor) -- but only when this really
# is a terminal, since `docker exec -it` refuses to run from a pipe or a CI job.
TTY_FLAGS=()
[[ -t 0 && -t 1 ]] && TTY_FLAGS=(-it)
# sudo because the workspace is root-only inside the container.
docker exec "${TTY_FLAGS[@]}" "$CONTAINER" sudo python3 \
  "/root/ament_ws/src/interfacing/can/scripts/${TOOL}" "$@" || RC=$?

AFTER="$(ls -1 "${REPO}/outputs/gl40_bench" 2>/dev/null | wc -l)"
if [[ "${GL40_NO_PLOT:-0}" == "1" || "$AFTER" == "$BEFORE" ]]; then
  exit "${RC:-0}"
fi

RUN="${REPO}/outputs/gl40_bench/$(ls -1t "${REPO}/outputs/gl40_bench" | head -1)"
echo
echo "=> plotting ${RUN}"
uv run --with matplotlib --with numpy "${REPO}/tools/gl40_telemetry_plot.py" "$RUN"
exit "${RC:-0}"
