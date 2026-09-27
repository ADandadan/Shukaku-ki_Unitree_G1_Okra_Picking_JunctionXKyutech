#!/usr/bin/env bash
# READ-ONLY look inside the G1 dev PC (allowed by the safety guide, section 12).
# Nothing is installed or changed. Password when asked: 123
# Usage: bash robot_inventory.sh   -> writes robot_inventory.txt
HOST=${1:-unitree@192.168.123.164}
OUT="$(dirname "$0")/robot_inventory.txt"
ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 "$HOST" 'bash -s' <<'EOF' | tee "$OUT"
echo "=== host ==="; hostname; uname -a; uptime
echo; echo "=== network ==="; ip -4 -o addr | awk "{print \$2, \$4}"
echo; echo "=== disk / mem ==="; df -h / | tail -1; free -h | head -2
echo; echo "=== cameras ==="; ls -l /dev/video* 2>/dev/null
command -v v4l2-ctl >/dev/null && v4l2-ctl --list-devices 2>/dev/null
command -v rs-enumerate-devices >/dev/null && rs-enumerate-devices -s 2>/dev/null
echo; echo "=== usb ==="; lsusb 2>/dev/null
echo; echo "=== relevant processes ==="
ps aux | grep -E "teleimager|image_server|dex1|ros2|zed|realsense|livox|unitree" | grep -v grep
echo; echo "=== teleimager ==="; command -v teleimager-server || echo "teleimager-server not on PATH"
for f in ~/.config/teleimager/teleimager_server.yaml; do
  [ -f "$f" ] && { echo "--- $f (non-comment lines)"; grep -v "^\s*#" "$f" | grep -v "^\s*$"; }
done
echo; echo "=== home dir ==="; ls ~
echo; echo "=== python sdk ==="; python3 -c "import unitree_sdk2py,sys;print('unitree_sdk2py at',unitree_sdk2py.__file__)" 2>&1 | tail -1
EOF
echo; echo "Saved to $OUT"
