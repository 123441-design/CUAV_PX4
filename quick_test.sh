#!/bin/bash
# Quick PX4 SIH test — runs PX4 interactively so you can type commands
# Run this in your WSL terminal:
#   bash ~/PX4-test/quick_test.sh

echo "=== Starting PX4 SIH ==="
echo "Wait for 'pxh>' prompt, then type:"
echo ""
echo "  commander check"
echo "  commander arm -f"
echo "  commander takeoff"
echo "  listener vehicle_local_position"
echo "  height_commander status"
echo ""

cd ~/PX4-test/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 ./bin/px4
