#!/usr/bin/env bash
set -e  # 에러 나면 바로 종료

#########################
# 설정 (원하면 바꿔도 됨)
#########################

# cuRobo가 포함된 Isaac Sim 이미지 이름
IMAGE_NAME=${ISAAC_CUROBO_IMAGE:-isaac_curobo:image}

# Dockerfile 경로
DOCKERFILE_PATH=${CUROBO_DOCKERFILE:-./isaac_curobo.dockerfile}

# IsaacSim-ros_workspaces를 clone & build 할 위치
ROS_WS_DIR=${ISAAC_ROS_WS_DIR:-$HOME/IsaacSim-ros_workspaces}

#########################
# Step 1. cuRobo 포함 이미지 빌드
#########################

echo "========== [1/2] Building Isaac + cuRobo Docker image =========="
echo "  - IMAGE_NAME    : $IMAGE_NAME"
echo "  - DOCKERFILE    : $DOCKERFILE_PATH"

if [ ! -f "$DOCKERFILE_PATH" ]; then
  echo "ERROR: Dockerfile not found at $DOCKERFILE_PATH"
  exit 1
fi

docker build -t "$IMAGE_NAME" -f "$DOCKERFILE_PATH" .
echo "✅ Docker image built: $IMAGE_NAME"
echo

#########################
# Step 2. IsaacSim-ros_workspaces clone & build
#########################

echo "========== [2/2] Cloning & Building IsaacSim-ros_workspaces =========="
echo "  - ROS_WS_DIR    : $ROS_WS_DIR"

if [ ! -d "$ROS_WS_DIR" ]; then
  echo "[ROS WS] Cloning IsaacSim-ros_workspaces..."
  git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git "$ROS_WS_DIR"
else
  echo "[ROS WS] Directory already exists. Skipping clone."
fi

cd "$ROS_WS_DIR"

# 이미 빌드된 적이 있으면 스킵
if [ -d build_ws/humble/humble_ws/install ] && [ -d build_ws/humble/isaac_sim_ros_ws/install ]; then
  echo "[ROS WS] Seems already built (install dirs exist). Skipping build_ros.sh."
else
  echo "[ROS WS] Running build_ros.sh for Humble / Ubuntu 22.04 (Python 3.11)..."
  ./build_ros.sh -d humble -v 22.04
fi

echo "✅ IsaacSim-ros_workspaces 준비 완료."
echo
echo "🎉 설치 자동화 완료!"
echo "  - Docker image : $IMAGE_NAME"
echo "  - ROS WS       : $ROS_WS_DIR"
