#!/usr/bin/env bash
# run_test.sh: 호스트 OS 오염 없이 Docker 격리 환경에서 ROS 2 자동화 검증 수행
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo ">>> [Host Test] 06_ros2_node Docker 격리 검증 시작 <<<"
echo "============================================================"

# 1. 도커 이미지 빌드 (캐시 활용)
echo "[1/3] Docker 검증 이미지 확인/빌드 중..."
docker build -t ros2_node_test:latest "$SCRIPT_DIR"

# 2. 컨테이너 내부 빌드 및 런치 검증 실행
# --net=host : NUCLEO 물리 이더넷과 직결
# -v src:/ros2_ws/src:ro : 호스트 파일시스템 오염 원천 차단 (build/install 부산물 격리)
echo "[2/3] Docker 컨테이너 실행 및 host_bringup 패키지 테스트 구동..."
docker run --rm -it --net=host \
  -v "$SCRIPT_DIR/src:/ros2_ws/src:ro" \
  -w /ros2_ws \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  ros2_node_test:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && \
           colcon build --symlink-install && \
           source install/setup.bash && \
           ros2 launch host_bringup ahrs_test.launch.py"

echo "[3/3] >>> 모든 검증이 성공적으로 완료되었습니다! <<<"
