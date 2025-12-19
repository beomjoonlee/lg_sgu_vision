# Robot Workspace (`robot_ws`)

본 워크스페이스는 **ROS 2 Humble + MoveIt 2** 기반으로  
**UR20 로봇 제어, 경로 계획, 궤적 실행(CSV / Topic)** 을 담당합니다.

---

## 🛠️ Installation

설치 방법은 **Docker 환경(권장)** 과 **Local 환경** 두 가지로 나뉩니다.

---

## 1. Docker Environment (Recommended)

### 설치 및 빌드

```bash
cd lg_sgu_vision/robot_ws
./install.sh
```

### 실행 및 접속

```bash
./execute.sh
```

---

## 2. Local Environment

### 필수 요구 사항

- **ROS 2 Humble**  
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

- **MoveIt 2 (Humble)**  
  https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html

---

### 빌드 순서

```bash
cd lg_sgu_vision/robot_ws

vcs import src/ur --skip-existing   --input src/ur/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.${ROS_DISTRO}.repos

rosdep update
rosdep install --ignore-src --from-paths src -y

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🚀 Execution

> 아래 실행 방법은 **Docker 환경과 Local 환경 모두에 공통적으로 적용됩니다.**

### 공통 환경 변수 설정

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<사용할_ID_숫자>
```

---

## 1. Robot Driver Launch

### Fake Hardware

```bash
ros2 launch ur_robot_driver ur_with_camera_control.launch.py   ur_type:=ur20   robot_ip:=192.168.0.3   use_fake_hardware:=true   launch_rviz:=true
```

### Real Hardware

```bash
ros2 launch ur_robot_driver ur_with_camera_control.launch.py   ur_type:=ur20   robot_ip:=192.168.0.3   use_fake_hardware:=false   launch_rviz:=true
```

---

## 2. MoveIt 2 Launch

```bash
ros2 launch ur_moveit_config ur_with_camera_moveit.launch.py   ur_type:=ur20   description_file:=ur20_with_camera.urdf.xacro   use_sim_time:=false   launch_rviz:=true   launch_servo:=false
```

---

## 3. Trajectory Player

### CSV 실행

```bash
ros2 run trajectory_player csv_moveit_player
```

### Topic 실행

```bash
ros2 run trajectory_player topic_moveit_player
```
