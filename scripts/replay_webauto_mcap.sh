#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage:
  replay_webauto_mcap.sh --bag-dir /path/to/log_dir [--autoware-setup /path/to/setup.bash] [--rate 1.0] [--no-wait]
  replay_webauto_mcap.sh /path/to/log_dir

Starts:
  1. rosbridge websocket server
  2. lanelet bridge node
  3. rosbag2 playback for the topics used by the Godot simulator

Notes:
  - Start Godot before pressing Enter, or pass --no-wait if Godot is already running.
  - AUTOWARE_SETUP can also be provided via environment variable.
  - ROS_HOME and ROS_LOG_DIR default to /tmp so the script works without writing to ~/.ros.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

BAG_DIR=""
AUTOWARE_SETUP="${AUTOWARE_SETUP:-}"
RATE="${RATE:-1.0}"
WAIT_FOR_ENTER=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-dir)
      [[ $# -ge 2 ]] || die "--bag-dir requires a value"
      BAG_DIR="$2"
      shift 2
      ;;
    --autoware-setup)
      [[ $# -ge 2 ]] || die "--autoware-setup requires a value"
      AUTOWARE_SETUP="$2"
      shift 2
      ;;
    --rate)
      [[ $# -ge 2 ]] || die "--rate requires a value"
      RATE="$2"
      shift 2
      ;;
    --no-wait)
      WAIT_FOR_ENTER=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "$BAG_DIR" ]]; then
        BAG_DIR="$1"
        shift
      else
        die "unexpected argument: $1"
      fi
      ;;
  esac
done

[[ -n "$BAG_DIR" ]] || die "bag directory is required"
[[ -d "$BAG_DIR" ]] || die "bag directory not found: $BAG_DIR"

export ROS_HOME="${ROS_HOME:-/tmp/driving_game_ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/driving_game_ros_logs}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

set +u
source /opt/ros/humble/setup.bash

if [[ -n "$AUTOWARE_SETUP" ]]; then
  [[ -f "$AUTOWARE_SETUP" ]] || die "AUTOWARE_SETUP not found: $AUTOWARE_SETUP"
  # shellcheck disable=SC1090
  source "$AUTOWARE_SETUP"
fi
set -u

ros2 pkg prefix autoware_map_msgs >/dev/null 2>&1 || die \
  "autoware packages are not available. Set AUTOWARE_SETUP=/path/to/autoware/install/setup.bash"

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${BAG_PID:-}" ]]; then
    kill "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "${LANELET_PID:-}" ]]; then
    kill "$LANELET_PID" 2>/dev/null || true
    wait "$LANELET_PID" 2>/dev/null || true
  fi
  if [[ -n "${ROSBRIDGE_PID:-}" ]]; then
    kill "$ROSBRIDGE_PID" 2>/dev/null || true
    wait "$ROSBRIDGE_PID" 2>/dev/null || true
  fi
  exit "$exit_code"
}
trap cleanup EXIT INT TERM

echo "[replay] Starting rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  max_message_size:=50000000 \
  > >(sed 's/^/[rosbridge] /') 2>&1 &
ROSBRIDGE_PID=$!

echo "[replay] Starting lanelet bridge..."
python3 "$SCRIPT_DIR/lanelet_bridge_node.py" \
  > >(sed 's/^/[lanelet_bridge] /') 2>&1 &
LANELET_PID=$!

sleep 2

echo "[replay] Ready."
echo "[replay] Bag: $BAG_DIR"
echo "[replay] Rate: $RATE"
if [[ "$WAIT_FOR_ENTER" -eq 1 ]]; then
  read -r -p "[replay] Start Godot, then press Enter to begin playback... "
fi

ros2 bag play "$BAG_DIR" \
  --rate "$RATE" \
  --read-ahead-queue-size 1000 \
  --disable-keyboard-controls \
  --wait-for-all-acked 5000 \
  --topics \
    /map/vector_map \
    /tf \
    /tf_static \
    /control/command/control_cmd \
    /control/command/actuation_cmd \
    /control/command/gear_cmd \
    /planning/trajectory \
    /planning/scenario_planning/trajectory \
    /perception/object_recognition/objects \
    /perception/traffic_light_recognition/traffic_signals &
BAG_PID=$!
wait "$BAG_PID"
