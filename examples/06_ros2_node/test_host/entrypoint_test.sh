#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
colcon build
source /ros2_ws/install/setup.bash

export ZENOH_ROUTER_CONFIG_URI=/ros2_ws/src/host_bringup/config/router_config.json5
export RUST_LOG=zenoh=debug,rmw_zenoh_cpp=debug
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOHD_PID=$!

sleep 5

set +e
ros2 launch host_bringup ahrs_test.launch.py
RC=$?

kill -9 $ZENOHD_PID 2>/dev/null || true
exit $RC
