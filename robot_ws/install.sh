#!/usr/bin/env bash
set -e

IMAGE_NAME="ros_humble_builder:latest"
CONTAINER_NAME="ros_dev"

# 경로 설정
SRC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SRC_WS}/.." && pwd)"
MOVEIT_WS="${ROOT_DIR}/ws_moveit2_docker"
ROBOT_WS="${ROOT_DIR}/robot_ws_docker"

CLEAN=0
FORCE_MOVEIT=0
FORCE_ROBOT=0

# 인자 파싱
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --force-moveit) FORCE_MOVEIT=1 ;;
    --force-robot) FORCE_ROBOT=1 ;;
  esac
done

# 1. 이미지 빌드
echo "[1/6] Build Docker Image"
docker build -t "${IMAGE_NAME}" -f "${SRC_WS}/Dockerfile" "${SRC_WS}"

# 2. 호스트 워크스페이스 준비 (Clean 옵션 시 삭제)
if [ "${CLEAN}" -eq 1 ]; then
  echo "[CLEAN] Removing workspaces..."
  sudo rm -rf "${MOVEIT_WS}" "${ROBOT_WS}"
fi
mkdir -p "${MOVEIT_WS}/src" "${ROBOT_WS}/src"

# 3. 소스 코드 동기화 (rsync)
# --delete 옵션 제거: 권한 문제 방지 및 기존 파일 보존
echo "[2/6] Sync robot_ws source"
rsync -a \
  --exclude ".git" --exclude "__pycache__" --exclude "*.pyc" \
  "${SRC_WS}/src/" "${ROBOT_WS}/src/"

# 4. 컨테이너 실행 (존재하지 않거나 꺼져있으면 실행)
echo "[3/6] Ensure Container is Running"
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
  echo "Creating new container with GUI support..."
  docker run -d \
    --name "${CONTAINER_NAME}" \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${MOVEIT_WS}:/workspace/ws_moveit2" \
    -v "${ROBOT_WS}:/workspace/robot_ws" \
    "${IMAGE_NAME}" \
    sleep infinity
else
  if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
    echo "Starting existing container..."
    docker start "${CONTAINER_NAME}"
  fi
fi

# 5. ws_moveit2 설치 및 빌드
echo "[4/6] Setup ws_moveit2 (Inside Container)"
docker exec -t "${CONTAINER_NAME}" bash -c "
  set -e
  source /opt/ros/humble/setup.bash
  
  # 소스 다운로드 (없을 때만)
  mkdir -p /workspace/ws_moveit2/src
  cd /workspace/ws_moveit2/src
  if [ ! -d moveit2_tutorials ]; then
    git clone --branch humble https://github.com/ros-planning/moveit2_tutorials
  fi
  # 기존 파일이 있어도 업데이트/스킵
  vcs import < moveit2_tutorials/moveit2_tutorials.repos --skip-existing

  # [중요] 의존성 설치 (항상 실행) -> 여기서 ruckig, ompl 등이 설치됨
  echo 'Running rosdep check for MoveIt...'
  cd /workspace/ws_moveit2
  apt-get update
  rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

  # 빌드 (강제 옵션이거나 install 폴더가 없을 때만 수행)
  DO_BUILD=0
  if [ \"${FORCE_MOVEIT}\" -eq 1 ] || [ ! -f \"install/setup.bash\" ]; then
    DO_BUILD=1
  fi

  if [ \"\$DO_BUILD\" -eq 1 ]; then
    echo 'Building ws_moveit2...'
    colcon build --mixin release --parallel-workers 1
  else
    echo 'ws_moveit2 already built. Skipping compilation.'
  fi
"

# 6. robot_ws 빌드
echo "[5/6] Build robot_ws (Inside Container)"
docker exec -t "${CONTAINER_NAME}" bash -c "
  set -e
  source /opt/ros/humble/setup.bash
  source /workspace/ws_moveit2/install/setup.bash
  
  cd /workspace/robot_ws
  
  # 추가 repo import (UR Driver 등)
  if [ -d src/ur/Universal_Robots_ROS2_Driver ]; then
      vcs import src/ur --skip-existing --input src/ur/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
  fi

  # 의존성 설치 (항상 실행)
  apt-get update
  rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

  # 빌드
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
"

echo "[6/6] Done! Use ./execute.sh to enter."
