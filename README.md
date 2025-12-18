# LG SGU Vision Project

이 리포지토리는 **GTSP(Generalized Traveling Salesman Problem) 알고리즘을 활용한 최적 경로 생성**과 **ROS 2/MoveIt 2 기반의 로봇 매니퓰레이터 제어**를 통합한 시스템입니다.

시스템은 크게 경로 생성 및 시뮬레이션을 담당하는 `gtsp_trajectory`와 실제 로봇 제어를 담당하는 `robot_ws` 두 가지 핵심 모듈로 구성되어 있습니다.

## 📂 Repository Structure

### 1. gtsp_trajectory
GTSP 알고리즘을 사용하여 최적의 검사 경로를 생성하고, Isaac Sim 환경에서 이를 시뮬레이션하며 경로 데이터를 ROS 2 토픽으로 전송합니다.

* **주요 기능**:
    * **GTSP Path Planning**: GTSP 알고리즘을 통해 최적의 방문 순서 및 경로 생성
    * **Isaac Sim Integration**: 생성된 경로를 NVIDIA Isaac Sim 상에서 시각화 및 실행
    * **Topic Publishing**: 생성된 궤적(Trajectory) 정보를 ROS 2 메시지로 송신
* **자세한 설명**: [gtsp_trajectory README 바로가기](https://github.com/beomjoonlee/lg_sgu_vision/tree/main/gtsp_trajectory)

### 2. robot_ws (ROS 2 Workspace)
ROS 2 Humble 기반의 워크스페이스로, 실제 로봇(또는 가상 로봇)의 드라이버를 구동하고 MoveIt 2를 통해 전송받은 경로를 실행합니다.

* **주요 기능**:
    * **Robot Drivers**: Universal Robots (UR) 등 로봇 하드웨어 드라이버 실행
    * **MoveIt 2 Execution**: 모션 플래닝 프레임워크(MoveIt 2) 구동
    * **Trajectory Execution**: `gtsp_trajectory`에서 전송한 경로 토픽을 수신하여 로봇 제어 및 이동 수행
* **자세한 설명**: [robot_ws README 바로가기](https://github.com/beomjoonlee/lg_sgu_vision/tree/main/robot_ws)

---

## 🚀 Installation & Setup

이 프로젝트를 로컬 환경에 복제(Clone)하여 사용하려면 다음 명령어를 실행하십시오.

```bash
# 리포지토리 복제
git clone [https://github.com/beomjoonlee/lg_sgu_vision.git](https://github.com/beomjoonlee/lg_sgu_vision.git)

# 폴더 이동
cd lg_sgu_vision
