#!/bin/bash
# Create pty for px4 and feed commands
# Run in WSL: bash ~/PX4-test/run_px4_pty.sh

set -e
BUILD_DIR="$HOME/PX4-test/build/px4_sitl_default"
OUT="/tmp/px4_full_output.txt"

pkill -9 px4 2>/dev/null || true
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
sleep 1

# Use script to create a pty for px4
# Feed commands via a fifo
FIFO="/tmp/px4_cmd_fifo"
rm -f "$FIFO"
mkfifo "$FIFO"

# Reader: sends commands after delay
(
  sleep 15
  echo "" > "$FIFO"
  echo "listener vehicle_status" > "$FIFO"
  sleep 5
  echo "commander arm" > "$FIFO"
  sleep 5
  echo "commander takeoff" > "$FIFO"
  sleep 8
  echo "listener vehicle_local_position" > "$FIFO"
  sleep 5
  echo "height_commander status" > "$FIFO"
  sleep 3
  echo "shutdown" > "$FIFO"
  sleep 3
) &

cd "$BUILD_DIR"
export PX4_SYS_AUTOSTART=10040

# Run px4 in a pty, redirecting its stdin from the fifo
script -q -c "./bin/px4 < $FIFO" "$OUT" &
SCRIPT_PID=$!

echo "PX4 started via script, PID=$SCRIPT_PID"
echo "Waiting 60 seconds..."
sleep 60

# Check if done
kill $SCRIPT_PID 2>/dev/null || true
wait $SCRIPT_PID 2>/dev/null || true

echo "=== FULL OUTPUT ==="
cat "$OUT"
