# UR20 Camera Add-ons (New Files)

이 README는 아래 두 레포지토리의 기존 구성에 **카메라(links/meshes/launch/moveit/servo/srdf)** 지원을 위해 새로 추가된 파일 목록을 정리한 것입니다.

- Universal_Robots_ROS2_Description: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description  
- Universal_Robots_ROS2_Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver  

---

## 1) Universal_Robots_ROS2_Description 에 추가된 파일

### URDF / Xacro
- `Universal_Robots_ROS2_Description/urdf/ur20_with_camera.urdf.xacro`  
  - UR20 모델에 카메라 링크/조인트가 포함된 URDF(Xacro) 엔트리입니다.
- `Universal_Robots_ROS2_Description/urdf/camera.xacro`  
  - 카메라 링크/조인트/비주얼(메시) 정의를 모듈화한 Xacro입니다.

### Meshes
- `Universal_Robots_ROS2_Description/meshes/camera/`  
  - 카메라 시각화를 위한 메시(예: dae/stl 등) 리소스 디렉토리입니다.

---

## 2) Universal_Robots_ROS2_Driver 에 추가된 파일

### Driver Launch
- `Universal_Robots_ROS2_Driver/ur_robot_driver/launch/ur_with_camera_control.launch.py`  
  - 카메라 포함 URDF를 사용해 드라이버/컨트롤을 실행하기 위한 런치 파일입니다.

### MoveIt Launch
- `Universal_Robots_ROS2_Driver/ur_moveit_config/launch/ur_with_camera_moveit.launch.py`  
  - 카메라 포함 URDF/SRDF 구성을 기반으로 MoveIt을 실행하기 위한 런치 파일입니다.

### MoveIt Servo Config
- `Universal_Robots_ROS2_Driver/ur_moveit_config/config/ur_with_camera_servo.yaml`  
  - Servo 사용 시 카메라 포함 구성에 맞춘 파라미터(프레임, 토픽, 제한 등) 설정 파일입니다.

### SRDF / Xacro
- `Universal_Robots_ROS2_Driver/ur_moveit_config/srdf/ur_with_camera.srdf.xacro`  
  - 카메라 포함 모델에 대응하는 SRDF(Xacro) 엔트리입니다.
- `Universal_Robots_ROS2_Driver/ur_moveit_config/srdf/ur_with_camera_macro.srdf.xacro`  
  - 카메라 포함 SRDF 구성을 재사용 가능하도록 분리한 매크로 정의 파일입니다.
