#!/bin/bash
set -u

repo=/home/yyy/PX4-test
build="$repo/build/px4_sitl_default"
px4_port=/tmp/mylink_px4
stm_port=/tmp/mylink_stm
log=/tmp/mylink_offboard_sitl.log

cleanup() {
    kill "${stream_pid:-}" "${px4_pid:-}" "${socat_pid:-}" 2>/dev/null || true
    wait "${stream_pid:-}" "${px4_pid:-}" "${socat_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT

pkill -9 px4 2>/dev/null || true
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0 "$px4_port" "$stm_port" "$log"

socat -d -d PTY,raw,echo=0,link="$px4_port" PTY,raw,echo=0,link="$stm_port" 2> /tmp/mylink_socat.log &
socat_pid=$!
sleep 1

cd "$build"
PX4_SYS_AUTOSTART=10040 ./bin/px4 -d >"$log" 2>&1 </dev/null &
px4_pid=$!

for _ in $(seq 1 40); do
    grep -q "Startup script returned successfully" "$log" 2>/dev/null && break
    sleep 1
done
sleep 2

px4cmd() {
    local module="$1"
    shift
    "$build/bin/px4-$module" --instance 0 "$@"
}

px4cmd mylink_bridge start -d "$px4_port" -b 115200
sleep 1
px4cmd commander arm -f
sleep 2

python3 "$repo/test_mylink_offboard_stream.py" "$stm_port" 7 > /tmp/mylink_stream.log 2>&1 &
stream_pid=$!

sleep 3
echo "=== DURING_STREAM_VEHICLE_STATUS ==="
px4cmd listener vehicle_status -n 1
echo "=== DURING_STREAM_BRIDGE_STATUS ==="
px4cmd mylink_bridge status

wait "$stream_pid"
unset stream_pid

sleep 1
echo "=== AFTER_STREAM_VEHICLE_STATUS ==="
px4cmd listener vehicle_status -n 1
echo "=== AFTER_STREAM_BRIDGE_STATUS ==="
px4cmd mylink_bridge status
echo "=== STREAM_GENERATOR ==="
cat /tmp/mylink_stream.log
echo "=== RELEVANT_PX4_LOG ==="
grep -E "Offboard|offboard|Armed|Disarmed|mylink_bridge" "$log" | tail -80
