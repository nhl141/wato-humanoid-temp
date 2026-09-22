#!/usr/bin/env bash
# Drive the arm through the ROS pipeline (joint_command -> can_node -> CAN), recording
# telemetry for every motor, then plot it on the host.
#
# Unlike tools/gl40_move.sh (which talks to one drive over raw SocketCAN), this exercises the
# real command path: ArmPose -> clamp/velocity-limit/smooth -> MotorCmd -> CAN. That is what
# the clamp and velocity benchmarks are about.
#
#   tools/gl40_ros_move.sh --pose "-124,-40.5,-165.4,-88.7,14.1,0"   # 6 joints, degrees
#   tools/gl40_ros_move.sh --pose "0,0,0,0,0,200" --duration 20      # wrist past its limit
#   tools/gl40_ros_move.sh --pose "..." --rate 200                   # publisher rate test
#
# Env: JC_SERVICE (default joint_command), GL40_NO_PLOT=1 to skip plotting.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE="${JC_SERVICE:-joint_command}"
POSE=""
DURATION=15
RATE=50
LABEL="ros"
MOTORS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pose) POSE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    --motors) MOTORS="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ -z "$POSE" ]]; then
  echo "--pose is required: six command-frame angles in degrees, e.g." >&2
  echo "  --pose \"shoulder_pitch,shoulder_roll,shoulder_yaw,elbow_pitch,elbow_roll,wrist_pitch\"" >&2
  exit 2
fi

IFS=',' read -r SP SR SY EP ER WP <<< "$POSE"
CONTAINER="$(docker ps --filter "name=-${SERVICE}-" --format '{{.Names}}' | head -1)"
[[ -z "$CONTAINER" ]] && { echo "No running '${SERVICE}' container. ./watod up -d" >&2; exit 1; }

mkdir -p "${REPO}/outputs/gl40_bench"
ARM_POSE="{is_left: true, shoulder: {position: [${SP}, ${SR}, ${SY}]}, elbow: {position: [${EP}, ${ER}]}, wrist: {position: [${WP}]}, include_hand_pose: false}"

echo "=> recording ${DURATION}s while publishing ArmPose at ${RATE} Hz"
echo "   pose: ${ARM_POSE}"

REC_ARGS=(--duration "$DURATION" --label "$LABEL")
[[ -n "$MOTORS" ]] && REC_ARGS+=(--motors "$MOTORS")

docker exec -d "$CONTAINER" bash -lc "source /opt/watonomous/setup.bash; \
  timeout $((DURATION + 2)) ros2 topic pub -r ${RATE} /arm/joint_targets \
  common_msgs/msg/ArmPose '${ARM_POSE}'"

# /opt/humanoid_scripts, not /root/...: this container runs as the host user, who cannot
# traverse root's home directory.
docker exec "$CONTAINER" bash -lc "source /opt/watonomous/setup.bash; \
  python3 /opt/humanoid_scripts/telemetry_record.py ${REC_ARGS[*]}"

if [[ "${GL40_NO_PLOT:-0}" == "1" ]]; then exit 0; fi
RUN="${REPO}/outputs/gl40_bench/$(ls -1t "${REPO}/outputs/gl40_bench" | head -1)"
echo
echo "=> plotting ${RUN}"
uv run --with matplotlib --with numpy "${REPO}/tools/gl40_telemetry_plot.py" "$RUN"
