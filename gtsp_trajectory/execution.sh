#!/usr/bin/env bash
set -e

########################################
# 설정 (환경변수로 덮어쓸 수 있음)
########################################

# 설치 스크립트에서 만든 이미지 이름과 맞추세요
IMAGE_NAME=${ISAAC_CUROBO_IMAGE:-isaac_curobo:image}
CONTAINER_NAME=${ISAAC_CONTAINER_NAME:-isaac_curobo_container}

# IsaacSim-ros_workspaces 위치 (설치 스크립트와 동일하게)
ROS_WS_DIR=${ISAAC_ROS_WS_DIR:-$HOME/IsaacSim-ros_workspaces}

# 프로젝트 폴더 (이 스크립트가 있는 디렉토리)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Xauthority (GUI를 위해 필요)
XAUTH=${XAUTHORITY:-$HOME/.Xauthority}

########################################
# 존재 여부 체크
########################################

if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
  echo "[run_isaac_curobo] ERROR: Docker image '$IMAGE_NAME' not found."
  echo "  먼저 install 스크립트(이미지 빌드)를 실행했는지 확인하세요."
  exit 1
fi

if [ ! -d "$ROS_WS_DIR" ]; then
  echo "[run_isaac_curobo] WARNING: ROS workspace dir '$ROS_WS_DIR' 가 없습니다."
  echo "  IsaacSim-ros_workspaces 를 아직 빌드하지 않았을 수도 있습니다."
fi

########################################
# X11 설정 (GUI를 위해 필요)
########################################

# xhost 명령어로 Docker 컨테이너의 X11 접근 허용
if command -v xhost > /dev/null 2>&1; then
  echo "[run_isaac_curobo] X11 forwarding 설정 중..."
  xhost +local:docker > /dev/null 2>&1 || echo "  WARNING: xhost 설정 실패 (GUI가 안 뜰 수 있음)"
else
  echo "[run_isaac_curobo] WARNING: xhost 명령어를 찾을 수 없습니다. GUI가 필요하면 수동으로 'xhost +local:docker'를 실행하세요."
fi

if [ ! -f "$XAUTH" ]; then
  echo "[run_isaac_curobo] WARNING: Xauthority 파일 '$XAUTH' 를 찾을 수 없습니다."
  echo "  GUI가 안 뜰 수도 있습니다. (headless라면 무시해도 됨)"
fi

########################################
# 기존 컨테이너 처리
########################################

if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
  # 컨테이너가 실행 중인지 확인
  if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
    echo "[run_isaac_curobo] 컨테이너 '$CONTAINER_NAME'가 이미 실행 중입니다."
    echo "  -> 기존 컨테이너에 접속합니다."
    docker exec -it "$CONTAINER_NAME" bash
    exit 0
  else
    echo "[run_isaac_curobo] 컨테이너 '$CONTAINER_NAME'가 멈춰있습니다."
    echo "  -> 컨테이너를 다시 시작하고 접속합니다."
    docker start "$CONTAINER_NAME"
    docker attach "$CONTAINER_NAME"
    exit 0
  fi
fi

########################################
# docker run (새 컨테이너 생성)
########################################

echo "[run_isaac_curobo] 새 컨테이너 생성:"
echo "  IMAGE    : $IMAGE_NAME"
echo "  NAME     : $CONTAINER_NAME"
echo "  ROS WS   : $ROS_WS_DIR"
echo "  PROJECT  : $PROJECT_DIR -> /curobo/gtsp_trajectory"

docker run --name "$CONTAINER_NAME" --entrypoint bash -it\
  --runtime=nvidia --gpus all \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  --network host \
  -e DISPLAY \
  -v "$XAUTH":/root/.Xauthority \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  -v "$ROS_WS_DIR":/workspace/IsaacSim-ros_workspaces:rw \
  -v "$PROJECT_DIR":/curobo/gtsp_trajectory:rw \
  $IMAGE_NAME