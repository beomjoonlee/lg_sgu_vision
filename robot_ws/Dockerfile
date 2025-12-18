# Dockerfile
FROM ubuntu:22.04
SHELL ["/bin/bash","-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1. 기본 툴 및 ROS2 레포지토리 설정
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  > /etc/apt/sources.list.d/ros2.list

# 2. ROS2 Base + 빌드 도구 설치
# (개별 라이브러리는 install.sh의 rosdep 단계에서 설치됩니다)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    python3-rosdep python3-vcstool \
    python3-colcon-common-extensions python3-colcon-mixin \
    git build-essential cmake rsync \
    && rm -rf /var/lib/apt/lists/*

# 3. rosdep 초기화
RUN rosdep init 2>/dev/null || true && rosdep update

# 4. colcon mixin (빌드 옵션 편의 기능)
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true \
 && colcon mixin update default
 
# 5. 접속할 때마다 자동으로 source 실행 (.bashrc에 추가)
# setup.bash 파일이 존재할 때만 source 하도록 조건문 추가 (에러 방지)
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /workspace/ws_moveit2/install/setup.bash ]; then source /workspace/ws_moveit2/install/setup.bash; fi' >> /root/.bashrc && \
    echo 'if [ -f /workspace/robot_ws/install/setup.bash ]; then source /workspace/robot_ws/install/setup.bash; fi' >> /root/.bashrc

WORKDIR /workspace
CMD ["sleep", "infinity"]
