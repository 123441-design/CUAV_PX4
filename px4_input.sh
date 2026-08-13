#!/bin/bash
# PX4 SITL interactive test script to be sourced by script command
# This runs inside the script pty

LOG="/tmp/px4_test_output.log"

# Commands to feed to pxh shell (with delays)
echo "=== WAITING FOR SHELL ===" >> "$LOG"
sleep 10
echo "=== SENDING COMMANDS ===" >> "$LOG"

# We send commands by echoing them to the terminal
echo "listener vehicle_status"
sleep 3

echo "commander arm"
sleep 4

echo "commander takeoff"
sleep 6

echo "listener vehicle_local_position"
sleep 3

echo "height_commander status"
sleep 2

echo "shutdown"
sleep 5

echo "=== TEST DONE ===" >> "$LOG"
