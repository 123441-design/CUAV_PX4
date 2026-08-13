#!/bin/bash
set -eu

repo=/home/yyy/PX4-test
build="$repo/build/px4_sitl_default"
px4_port=/tmp/mylink_motor_px4
stm_port=/tmp/mylink_motor_stm
log=/tmp/mylink_motor_sitl.log
ulog_result=/tmp/mylink_motor_ulog_result.log

cleanup() {
    for pid in ${stream_pid:-} ${px4_pid:-} ${socat_pid:-}; do
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

pkill -9 px4 2>/dev/null || true
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0 "$px4_port" "$stm_port" "$log" \
    "$ulog_result"

socat -d -d PTY,raw,echo=0,link="$px4_port" PTY,raw,echo=0,link="$stm_port" 2>/tmp/mylink_motor_socat.log &
socat_pid=$!
sleep 1

cd "$build"
PX4_SIM_SPEED_FACTOR=10 PX4_SYS_AUTOSTART=10040 ./bin/px4 -d >"$log" 2>&1 </dev/null &
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

python3 "$repo/test_mylink_offboard_stream.py" "$stm_port" 45 --motor-test >/tmp/mylink_motor_stream.log 2>&1 &
stream_pid=$!

# One command is injected at t=1.8 s and expires at t=4.8 s. The firmware must
# continuously maintain the output until expiry without command retransmission.
wait "$stream_pid"
unset stream_pid

px4cmd mylink_bridge status >/tmp/mylink_motor_status.log 2>&1
px4cmd commander disarm -f >/dev/null 2>&1 || true
kill "$px4_pid" 2>/dev/null || true
wait "$px4_pid" 2>/dev/null || true
unset px4_pid

ulog=$(find "$build/rootfs/log" -name '*.ulg' -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)
python3 "$repo/test_mylink_motor_ulog.py" "$ulog" >"$ulog_result"

grep -q "MOTOR_TEST_ACK=0" /tmp/mylink_motor_stream.log
grep -Eq "event=1 active=no function=0 throttle=0.0% commands=1 invalid=0 setpoints=[1-9][0-9]* releases=1" /tmp/mylink_motor_status.log
grep -q "gate_dropped=1" /tmp/mylink_motor_status.log
grep -q "RESULT PASS_MYLINK_MOTOR_ULOG" "$ulog_result"

echo "RESULT PASS_MYLINK_MOTOR_THROTTLE_SITL"
cat /tmp/mylink_motor_stream.log
cat /tmp/mylink_motor_status.log
cat "$ulog_result"
