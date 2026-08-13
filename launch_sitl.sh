#!/bin/bash
# PX4 Gazebo SITL launcher
# Run this in WSL terminal: bash ~/PX4-test/launch_sitl.sh

cd ~/PX4-test/build/px4_sitl_default

export PX4_NET_INTERFACE=eth0
export HEADLESS=1
export PX4_SYS_AUTOSTART=4001
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLDS="$HOME/PX4-test/Tools/simulation/gz/worlds"
export PX4_GZ_MODELS="$HOME/PX4-test/Tools/simulation/gz/models"
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-test/Tools/simulation/gz/models"
export PATH="./bin:$PATH"

echo "=== Starting PX4 SITL + Gazebo gz_x500 ==="
echo "MAVLink: eth0 broadcast"
echo "Gazebo: headless"
echo ""

./bin/px4 -d
