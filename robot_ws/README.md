# Robot Workspace (`robot_ws`)

본 워크스페이스는 **ROS 2 Humble + MoveIt 2** 기반으로  
**UR20 로봇 제어, 경로 계획, 궤적 실행(CSV / Topic)** 을 담당합니다.

이 워크스페이스는 다음과 같은 목적을 가지고 있습니다.

- UR20 로봇을 ROS 2 환경에서 제어
- 실제 로봇 없이도 테스트 가능한 Fake Hardware 시뮬레이션 제공
- MoveIt 2를 이용한 경로 계획 및 시각화
- CSV 파일 또는 ROS Topic으로부터 궤적을 입력받아 실행

---

## 🛠️ Installation

본 워크스페이스는 **Docker 환경**과 **Local 환경** 모두를 지원합니다.  
Docker 환경은 의존성 충돌을 최소화할 수 있어 권장됩니다.

---

## 1. Docker Environment (Recommended)

Docker를 사용하면 ROS 2, MoveIt 2, UR Driver가
미리 구성된 컨테이너 환경에서 실행됩니다.

### 📦 설치 및 빌드

아래 스크립트는 Docker 이미지를 빌드하고 컨테이너를 생성합니다.

```bash
cd lg_sgu_vision/robot_ws
./install.sh
```

### ▶️ 실행 및 접속

아래 명령은 생성된 컨테이너에 접속하며,
ROS 2 및 MoveIt 2 환경이 자동으로 설정됩니다.

```bash
./execute.sh
```

---

## 2. Local Environment

로컬 PC에 ROS 2 및 관련 패키지를 직접 설치하여 사용하는 방식입니다.
Docker 사용이 어려운 경우에만 권장됩니다.

### ✅ 필수 요구 사항

- **ROS 2 Humble**  
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

- **MoveIt 2 (Humble)**  
  https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

---

### 🔧 빌드 순서

#### 1️⃣ 워크스페이스 이동

```bash
cd lg_sgu_vision/robot_ws
```

#### 2️⃣ 의존성 리포지토리 가져오기

UR20 로봇 제어를 위한 Universal Robots ROS 2 Driver를 가져옵니다.

```bash
vcs import src/ur --skip-existing   --input src/ur/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.${ROS_DISTRO}.repos
```

#### 3️⃣ 의존성 패키지 설치

```bash
rosdep update
rosdep install --ignore-src --from-paths src -y
```

#### 4️⃣ 빌드

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 5️⃣ 환경 변수 설정

터미널을 열 때마다 본 워크스페이스가 자동으로 설정되도록 합니다.

```bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Execution

아래 실행 절차는 **Docker 환경과 Local 환경 모두에 공통적으로 적용**됩니다.

### 🔧 공통 환경 변수 설정

ROS 2 통신 미들웨어와 Domain ID를 설정합니다.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<사용할_ID_숫자>
```

---

## 1. Robot Driver Launch

UR20 로봇과의 실제 통신을 담당하는 드라이버를 실행합니다.
Fake Hardware 모드와 Real Hardware 모드를 모두 지원합니다.

### 🤖 Fake Hardware 모드

실제 로봇 없이 RViz 상에서 동작만 확인할 때 사용합니다.

```bash
ros2 launch ur_robot_driver ur_with_camera_control.launch.py   ur_type:=ur20   robot_ip:=192.168.0.3   use_fake_hardware:=true   launch_rviz:=true
```

### 🦾 Real Hardware 모드

실제 UR20 로봇과 연결하여 제어할 때 사용합니다.

```bash
ros2 launch ur_robot_driver ur_with_camera_control.launch.py   ur_type:=ur20   robot_ip:=192.168.0.3   use_fake_hardware:=false   launch_rviz:=true
```

---

## 2. MoveIt 2 Launch

MoveIt 2를 실행하여 경로 계획, 목표 자세 설정,
RViz 기반 시각화를 수행합니다.

```bash
ros2 launch ur_moveit_config ur_with_camera_moveit.launch.py   ur_type:=ur20   description_file:=ur20_with_camera.urdf.xacro   use_sim_time:=false   launch_rviz:=true   launch_servo:=false
```

---

## 3. Trajectory Player

계획된 궤적을 실제 로봇 또는 시뮬레이션에 실행하는 노드입니다.

### 📄 CSV 기반 실행

아래 경로의 CSV 파일을 읽어 순차적으로 궤적을 실행합니다.

- csv파일 경로:
```
robot_ws/src/trajectory_player/trajectory_player/trajectory.csv
```

- 실행:
```bash
ros2 run trajectory_player csv_moveit_player
```

### 📡 Topic 기반 실행

외부 노드에서 퍼블리시되는 trajectory 토픽을 수신하여 실행합니다.

```bash
ros2 run trajectory_player topic_moveit_player
```

## 📌 UR 공식 레포지토리 대비 추가/확장 내용 안내

본 워크스페이스에는  
- Universal_Robots_ROS2_Description: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description  
- Universal_Robots_ROS2_Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver  
에 **UR20에 카메라를 통합한 구성**을 추가하였습니다.

기존 UR 공식 레포지토리 대비 **추가된 파일 목록과 각 파일의 역할에 대한 상세 설명은**  
아래 경로의 README 문서에 정리되어 있습니다.

https://github.com/beomjoonlee/lg_sgu_vision/tree/main/robot_ws/src/ur

