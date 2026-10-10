#!/usr/bin/env bash
# rviz2_view.sh: 호스트 X11 디스플레이를 통해 컨테이너 내부 RViz2 3D 시각화 구동
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo ">>> [Host View] Docker 격리 환경에서 RViz2 3D 시각화 실행 <<<"
echo "============================================================"

# X11 권한 허용 (필요 시)
xhost +local:root 2>/dev/null || true

docker run --rm -it --net=host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$SCRIPT_DIR/src:/ros2_ws/src:ro" \
  -w /ros2_ws \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  ros2_node_test:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           colcon build --symlink-install && \
           source install/setup.bash && \
           ros2 launch host_bringup ahrs_view.launch.py"
