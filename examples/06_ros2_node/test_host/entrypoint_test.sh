#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
colcon build
source /ros2_ws/install/setup.bash

export ZENOH_ROUTER_CONFIG_URI=/ros2_ws/src/host_bringup/config/router_config.json5
export RUST_LOG=zenoh=warn,rmw_zenoh_cpp=warn
ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
ZENOHD_PID=$!

sleep 3

# [자동화 테스트 1] /nucleo/cmd_vel 매칭 구독자 확인 및 pub --once 검증 (타임아웃 5초)
echo "=== [Test Step 1] /nucleo/cmd_vel 매칭 구독자(Matching Subscription) 및 pub --once 검증 ==="
timeout 5s ros2 topic pub --once /nucleo/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
PUB_RC=$?
if [ $PUB_RC -ne 0 ]; then
  echo ">>> [ERROR] /nucleo/cmd_vel pub --once 매칭 구독자 타임아웃/실패 (RC=$PUB_RC) <<<"
  kill -9 $ZENOHD_PID 2>/dev/null || true
  cat /tmp/zenohd.log
  exit 1
fi
echo ">>> [PASS] /nucleo/cmd_vel pub --once 매칭 구독자 정상 식별 및 발행 완료 <<<"

# [자동화 테스트 2] 5대 센서 스트림, 양방향 cmd_vel/set_led E2E 정합성 검증
echo "=== [Test Step 2] AHRS 5대 센서 및 서비스/토픽 종합 E2E 검증 ==="
set +e
ros2 launch host_bringup ahrs_test.launch.py
RC=$?

kill -9 $ZENOHD_PID 2>/dev/null || true
if [ $RC -ne 0 ]; then
  echo "=== [DEBUG] Zenoh Router Log on Failure ==="
  cat /tmp/zenohd.log
fi
exit $RC
