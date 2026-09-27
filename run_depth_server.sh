#!/usr/bin/env bash
# Start robot_depth_server.py ON the G1 dev PC - READ-ONLY: camera -> ZMQ, no robot commands.
# Nothing is written on the robot: the script travels inside the ssh command (base64) and runs
# from memory with python -B. Ctrl-C here ends the ssh session, and the server exits and
# releases the camera.
# Usage: bash run_depth_server.sh [server args, e.g. --fps 30]      (password 123)
# Then, on the laptop: python detect_okra.py --source robot-depth
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
HOST=${ROBOT_HOST:-unitree@192.168.123.164}
PY=/home/unitree/miniconda3/envs/teleimager/bin/python   # has pyrealsense2 2.50, zmq, cv2
B64=$(base64 -w0 "$HERE/robot_depth_server.py")
ARGS=""
[ $# -gt 0 ] && ARGS=$(printf '%q ' "$@")   # printf with no args would emit '' (an empty arg)
# Keep ssh's stdin open with a sleeper (the server's watchdog exits on stdin EOF);
# kill the sleeper when ssh ends, whichever side stopped first.
exec 3< <(sleep infinity)
KEEP=$!
trap 'kill $KEEP 2>/dev/null || true' EXIT
ssh -o ConnectTimeout=5 "$HOST" \
  "exec $PY -B -c \"import base64; exec(compile(base64.b64decode('$B64'), 'robot_depth_server.py', 'exec'))\" $ARGS" <&3
