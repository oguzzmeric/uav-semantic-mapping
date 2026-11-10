#!/usr/bin/env bash
# Start PX4 SITL + Gazebo with the project airframe, from a clean parameter store.
#
# PX4 saves parameters it changes at run time (e.g. magnetometer offsets learned in flight)
# to rootfs/parameters.bson and loads them on the next start, so two runs would not start
# from the same state. All project settings live in the airframe file, so the store is
# deleted before every start.
#
# Usage: setup/run_sim.sh               # with the Gazebo GUI
#        setup/run_sim.sh HEADLESS=1    # extra NAME=value arguments become environment
set -eo pipefail  # no -u: the ROS setup script reads unset variables

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
rm -f "$PX4_DIR"/build/px4_sitl_default/rootfs/parameters*.bson

source /opt/ros/jazzy/setup.bash  # PX4 must run against ROS's Gazebo (8.10), the one ros_gz_bridge is built for
source "$HOME/px4-venv/bin/activate"
cd "$PX4_DIR"
# Render Gazebo on the NVIDIA GPU (PRIME on-demand)
exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia "$@" \
    make px4_sitl gz_usm_x500_gimbal
