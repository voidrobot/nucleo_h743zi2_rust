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
# ARP 캐시 워밍업 (타깃 보드와 호스트 상호 인식)
ping -c 1 -W 1 192.168.50.18 >/dev/null 2>&1 || true

chmod +x "$SCRIPT_DIR/entrypoint_test.sh"
docker run --rm --net=host \
  -v "$SCRIPT_DIR/src:/ros2_ws/src:ro" \
  -v "$SCRIPT_DIR/entrypoint_test.sh:/ros2_ws/entrypoint_test.sh:ro" \
  -w /ros2_ws \
  -e PYTHONUNBUFFERED=1 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  ros2_node_test:latest \
  bash /ros2_ws/entrypoint_test.sh

echo "[3/3] >>> 모든 검증이 성공적으로 완료되었습니다! <<<"
