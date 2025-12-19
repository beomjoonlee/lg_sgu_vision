# GTSP Trajectory
## 주요 기능

- 검사 대상 표면 자동 추출 및 뷰포인트 생성
- cuRobo 기반 충돌 없는 로봇 궤적 계획
- Isaac Sim 시뮬레이션 환경
- ROS2 연동

## 시스템 요구사항

- **OS**: Ubuntu 22.04
- **GPU**: NVIDIA GPU (CUDA 지원 필수)
- **Software**:
  - Docker / NVIDIA Container Toolkit
  - Isaac Sim 5.0 / cuRobo

## 설치 방법

### 1. 사전 준비

Docker, NVIDIA Container Toolkit을 설치합니다:

```bash
# 공식 가이드 참고
https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html
```

### 2. Docker 이미지 빌드 (최초 1회)

자동 설치 스크립트를 실행합니다:

```bash
./install.sh
```

**역할:** Isaac Sim + cuRobo Docker 이미지 빌드 (isaac_curobo:image)

**소요 시간:** 첫 빌드 시 30분~1시간 이상 소요될 수 있습니다.

## Docker 컨테이너 실행 (매 실행 시)

```bash
./execution.sh
```

### 컨테이너 내부 공통 환경 변수 설정

ROS 2 통신 미들웨어와 Domain ID를 설정합니다.
(robot_ws와 동일하게 모든 터미널에서 설정해주셔야 합니다.)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<사용할_ID_숫자>
```

### 스크립트 실행
```bash
# 프로젝트 디렉토리로 이동
cd /curobo/gtsp_trajectory

# 비전 검사 스크립트 실행
omni_python scripts/1_create_viewpoint.py \
    --object sample --material-rgb "170,163,158"
```

### 사용 방법

구체적인 비전 검사 파이프라인 사용 방법은 [scripts/README.md](scripts/README.md)를 참고해주세요.


## 참고: Docker 이미지/컨테이너 재빌드(클린 빌드)

### 언제 재빌드가 필요한가?
- Dockerfile 변경 (패키지 추가/환경변수/시스템 라이브러리 등)
- cuRobo / Isaac Sim 버전 변경 또는 의존성 설치 방식 변경
- `ur20_description/` 등 로봇 모델(URDF/mesh) 변경
- 컨테이너 내부에 설치한 내용이 꼬였거나(의존성 충돌) 깔끔한 초기화가 필요한 경우

### 클린 재빌드 절차
```bash
docker stop isaac_curobo_container # 컨테이너 중지
docker rm isaac_curobo_container   # 컨테이너 삭제
docker rmi isaac_curobo:image      # 이미지 삭제

./install.sh      # 이미지 재설치
./execution.sh    # 컨테이너 실행
```