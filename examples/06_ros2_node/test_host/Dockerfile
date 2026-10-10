# Dockerfile: ROS 2 Jazzy + rmw_zenoh_cpp 최소 테스트 런타임
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ROS_DOMAIN_ID=0

# rmw_zenoh_cpp 및 rviz2(선택 실행용) 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 워크스페이스 디렉토리 생성
WORKDIR /ros2_ws

# 컨테이너 시작 시 ROS 2 환경 로드
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
