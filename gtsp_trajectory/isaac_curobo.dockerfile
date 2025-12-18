FROM nvcr.io/nvidia/isaac-sim:5.0.0

# root 권한으로 실행
USER root

# 기본 파일 설치
RUN apt-get update && \
    apt-get install -y \
    vim \
    nano \
    wget \
    curl \
    git \
    git-lfs \
    libeigen3-dev

# cuda toolkit 12.8 설치
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
    mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    wget https://developer.download.nvidia.com/compute/cuda/12.8.0/local_installers/cuda-repo-ubuntu2204-12-8-local_12.8.0-570.86.10-1_amd64.deb && \
    dpkg -i cuda-repo-ubuntu2204-12-8-local_12.8.0-570.86.10-1_amd64.deb && \
    cp /var/cuda-repo-ubuntu2204-12-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get -y install cuda-toolkit-12-8


# cuRobo 설치

# 환경변수 설정
ENV PATH=${PATH}:/usr/local/cuda-12.8/bin
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-12.8/lib64

# GPU architecture 지정 (사용하는 GPU에 맞게 조정, RTX 50 series)
ENV TORCH_CUDA_ARCH_LIST="12.0"

RUN echo 'export PATH=${PATH}:/usr/local/cuda-12.8/bin' >> /root/.bashrc && \
    # echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-12.8/lib64' >> /root/.bashrc && \
    echo "alias omni_python='/isaac-sim/python.sh'" >> /root/.bashrc

RUN /isaac-sim/python.sh -m pip install tomli wheel ninja

WORKDIR /
RUN git clone https://github.com/NVlabs/curobo.git
WORKDIR /curobo

RUN /isaac-sim/python.sh -m pip install -e ".[isaacsim]" --no-build-isolation
RUN /isaac-sim/python.sh -m pip install open3d EAIK h5py

# 로컬 파일 복사 (robot description)
COPY ur20_description/ur20_with_camera.yml /curobo/src/curobo/content/configs/robot/
COPY ur20_description/ur20_with_camera.urdf /curobo/src/curobo/content/assets/robot/ur_description/
COPY ur20_description/ur20 /curobo/src/curobo/content/assets/robot/ur_description/meshes/ur20
COPY ur20_description/camera /curobo/src/curobo/content/assets/robot/ur_description/meshes/camera

# 프로젝트 작업 디렉토리 생성 (실제 파일은 volume mount로 연결됨)
WORKDIR /curobo/gtsp_trajectory

# ROS2 FastDDS 설정 파일 복사
COPY fastdds.xml /root/.ros/fastdds.xml

# Isaac + ROS 환경변수 및 ROS workspace source를 bashrc에 추가
RUN echo 'export ISAAC_SIM_PACKAGE_PATH=/isaac-sim' >> /root/.bashrc && \
    echo 'export ROS_DISTRO=humble' >> /root/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ISAAC_SIM_PACKAGE_PATH}/exts/isaacsim.ros2.bridge/humble/lib' >> /root/.bashrc && \
    echo 'export COLCON_PYTHON_EXECUTABLE=${ISAAC_SIM_PACKAGE_PATH}/python.sh' >> /root/.bashrc && \
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/root/.ros/fastdds.xml}' >> /root/.bashrc && \
    echo 'export ROS_WS_ROOT=${ROS_WS_ROOT:-/workspace/IsaacSim-ros_workspaces}' >> /root/.bashrc && \
    echo 'source "${ROS_WS_ROOT}/build_ws/humble/humble_ws/install/local_setup.bash"' >> /root/.bashrc && \
    echo 'source "${ROS_WS_ROOT}/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash"' >> /root/.bashrc && \
    echo '' >> /root/.bashrc