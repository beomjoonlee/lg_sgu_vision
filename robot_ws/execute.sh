#!/usr/bin/env bash
CONTAINER_NAME="ros_dev"

# 컨테이너 실행 확인
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
  echo "Container '${CONTAINER_NAME}' is not running."
  echo "Please run ./install.sh first."
  exit 1
fi

# 접속 (모든 워크스페이스 source 적용)
docker exec -it "${CONTAINER_NAME}" bash -lc "
  export ROS_DISTRO=humble
  source /opt/ros/humble/setup.bash
  
  if [ -f /workspace/ws_moveit2/install/setup.bash ]; then
    source /workspace/ws_moveit2/install/setup.bash
  fi
  if [ -f /workspace/robot_ws/install/setup.bash ]; then
    source /workspace/robot_ws/install/setup.bash
  fi
  
  cd /workspace/robot_ws
  exec bash
"
