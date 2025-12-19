#!/usr/bin/env python3
import csv
import time
import numpy as np
from pathlib import Path
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
    CollisionObject,  # 장애물 추가용
)


class CsvMoveitPlayer(Node):
    """
    1) 장애물(Table, Walls)을 MoveIt Planning Scene에 등록
    2) MoveIt2의 /plan_kinematic_path 서비스를 이용해서
       현재 joint 상태 -> CSV 첫 joint 상태까지 모션 플래닝 (장애물 회피)
       → 플래닝 결과 trajectory를 실행
    3) 그 이후 CSV joint trajectory 전체를
       → FollowJointTrajectory로 실행
    """

    def __init__(self) -> None:
        super().__init__("csv_moveit_player")

        # ---- 공통 dt(초) 설정: 모든 waypoint 간격을 0.01초로 ----
        self._dt = 0.01

        # --- Parameters ---
        current_path = Path(__file__).resolve()
        workspace_root = None
        for parent in current_path.parents:
            if parent.name == "robot_ws":
                workspace_root = parent
                break
        if workspace_root:
            default_csv_path = workspace_root / "src/trajectory_player/trajectory_player/trajectory.csv"
        else:
            self.get_logger().warn("'robot_ws' directory not found in path! Fallback to default.")
            default_csv_path = Path("/workspace/robot_ws/src/trajectory_player/trajectory_player/trajectory.csv")
        
        self.get_logger().info(f"[trajectory_player] Default CSV path set to: {default_csv_path}")
        
        self.declare_parameter("csv_path", str(default_csv_path))
        self.declare_parameter("group_name", "ur_manipulator")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        # 장애물 기준 프레임
        self.declare_parameter("planning_frame", "base_link")

        self._csv_path = Path(self.get_parameter("csv_path").value)
        self._group_name: str = self.get_parameter("group_name").value
        self._controller_name: str = self.get_parameter("controller_name").value
        self._joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self._planning_frame: str = self.get_parameter("planning_frame").value

        self.get_logger().info(f"[trajectory_player] CSV path: {self._csv_path}")
        self.get_logger().info(f"[trajectory_player] MoveIt group: {self._group_name}")
        self.get_logger().info(f"[trajectory_player] Planning Frame: {self._planning_frame}")

        # --- Collision Object Publisher (장애물 추가용) ---
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", qos_profile
        )

        # --- 장애물 추가 실행 (MoveIt 연결 확인 포함) ---
        self._publish_obstacles()

        # --- Load CSV (time + joints) ---
        self._trajectory: List[Tuple[float, List[float]]] = self._load_csv(self._csv_path)
        if not self._trajectory:
            raise RuntimeError("CSV에서 trajectory를 한 줄도 읽지 못했습니다.")

        # --- 현재 joint state 구독 ---
        self._current_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        # --- Action Client (FollowJointTrajectory) ---
        fjt_action_name = f"/{self._controller_name}/follow_joint_trajectory"
        self._fjt_client = ActionClient(
            self,
            FollowJointTrajectory,
            fjt_action_name,
        )
        self.get_logger().info(f"[trajectory_player] FJT Action server: {fjt_action_name}")

        # --- Service Client (GetMotionPlan) ---
        self._plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
        )
        self.get_logger().info("[trajectory_player] Motion plan service: /plan_kinematic_path")

    # ------------------------------------------------------------------
    # 장애물 추가 및 전송 (Wait for Subscriber 포함)
    # ------------------------------------------------------------------
    def _publish_obstacles(self):
        # 1. 장애물 데이터 정의
        TABLE = {
            "name": "table",
            "position": np.array([0.0, 1.09 + 0.25, 0.365 - 0.8], dtype=np.float64),
            "dimensions": np.array([1.0, 0.6, 0.73], dtype=np.float64),
        }

        WALLS = [
            {
                "name": "wall_front",
                "position": np.array([0.0, 1.6, 0.5], dtype=np.float64),
                "dimensions": np.array([2.2, 0.1, 3.0], dtype=np.float64),
            },
            {
                "name": "wall_back",
                "position": np.array([0.0, -1.0, 0.5], dtype=np.float64),
                "dimensions": np.array([2.2, 0.1, 3.0], dtype=np.float64),
            },
            {
                "name": "wall_left",
                "position": np.array([-1.0, 0.25, 0.5], dtype=np.float64),
                "dimensions": np.array([0.1, 2.7, 3.0], dtype=np.float64),
            },
            {
                "name": "wall_right",
                "position": np.array([1.0, 0.25, 0.5], dtype=np.float64),
                "dimensions": np.array([0.1, 2.7, 3.0], dtype=np.float64),
            },
            {
                "name": "support",
                "position": np.array([0.0, 1.22, -0.0525], dtype=np.float64),
                "dimensions": np.array([0.1, 0.1, 0.265], dtype=np.float64),
            },
        ]

        all_obstacles = [TABLE] + WALLS

        # [중요] MoveIt이 토픽을 구독할 때까지 대기
        self.get_logger().info("MoveIt(/collision_object) 연결 대기 중...")
        while self._collision_pub.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("MoveIt 연결 확인됨. 장애물 전송 시작.")

        for obs_data in all_obstacles:
            collision_object = CollisionObject()
            collision_object.header.frame_id = self._planning_frame
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = obs_data["name"]
            collision_object.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(obs_data["dimensions"][0]),
                float(obs_data["dimensions"][1]),
                float(obs_data["dimensions"][2]),
            ]

            pose = Pose()
            pose.position.x = float(obs_data["position"][0])
            pose.position.y = float(obs_data["position"][1])
            pose.position.z = float(obs_data["position"][2])
            pose.orientation.w = 1.0

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)

            self._collision_pub.publish(collision_object)
            time.sleep(0.05) # 패킷 뭉침 방지용 딜레이
        
        self.get_logger().info(f"총 {len(all_obstacles)}개의 장애물 전송 완료.")

    # ------------------------------------------------------------------
    # JointState 콜백
    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg: JointState) -> None:
        self._current_joint_state = msg

    # ------------------------------------------------------------------
    # CSV 로드
    # ------------------------------------------------------------------
    def _load_csv(self, path: Path) -> List[Tuple[float, List[float]]]:
        if not path.exists():
            self.get_logger().error(f"CSV 파일이 존재하지 않습니다: {path}")
            return []

        traj: List[Tuple[float, List[float]]] = []
        with path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            if "time" not in reader.fieldnames:
                raise RuntimeError("CSV 헤더에 'time' 컬럼이 없습니다.")

            for row in reader:
                t = float(row["time"])
                positions = [float(row[name]) for name in self._joint_names]
                traj.append((t, positions))

        self.get_logger().info(f"CSV에서 {len(traj)} 개의 trajectory point를 읽었습니다.")
        return traj

    # ------------------------------------------------------------------
    # float 초 → Duration 변환
    # ------------------------------------------------------------------
    @staticmethod
    def _to_duration(t: float) -> Duration:
        sec = int(t)
        nanosec = int(round((t - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    # ------------------------------------------------------------------
    # MoveIt2: 현재 상태 → CSV 첫 포인트까지 플래닝 (장애물 고려됨)
    # ------------------------------------------------------------------
    def _plan_and_execute_to_first_point(self) -> bool:
        first_time, first_positions = self._trajectory[0]
        self.get_logger().info(
            f"[trajectory_player] MoveIt2로 시작 위치로 이동합니다. (t={first_time})"
        )

        # 1) joint_states 수신 대기
        wait_cnt = 0
        while rclpy.ok() and self._current_joint_state is None and wait_cnt < 100:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_cnt += 1

        if self._current_joint_state is None:
            self.get_logger().error("joint_states를 받지 못했습니다. (timeout)")
            return False

        # 2) motion plan service 준비 대기
        if not self._plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/plan_kinematic_path 서비스를 찾지 못했습니다.")
            return False

        # 3) GetMotionPlan 요청 생성
        req = GetMotionPlan.Request()
        mreq: MotionPlanRequest = req.motion_plan_request

        mreq.group_name = self._group_name
        mreq.num_planning_attempts = 1
        mreq.allowed_planning_time = 5.0
        mreq.max_velocity_scaling_factor = 0.1
        mreq.max_acceleration_scaling_factor = 0.1

        start_state = RobotState()
        start_state.joint_state = self._current_joint_state
        mreq.start_state = start_state

        constraints = Constraints()
        for name, pos in zip(self._joint_names, first_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mreq.goal_constraints.append(constraints)

        self.get_logger().info("경로 플래닝 중 (장애물 회피 포함)...")
        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error("plan_kinematic_path 서비스 호출 실패")
            return False

        res = future.result()
        error_code = res.motion_plan_response.error_code.val

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveIt 플래닝 실패. error_code={error_code}")
            return False

        traj = res.motion_plan_response.trajectory.joint_trajectory
        self.get_logger().info(f"플래닝 성공. 경로 길이: {len(traj.points)}")

        if not traj.points:
            return False

        # 4) 실행
        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = traj

        if not self._fjt_client.wait_for_server(timeout_sec=10.0):
            return False

        send_future = self._fjt_client.send_goal_async(fjt_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("초기 이동 Goal 거부됨.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f"초기 위치 이동 완료. (Code: {result.error_code})")
        return True

    # ------------------------------------------------------------------
    # CSV 전체를 JointTrajectory 메시지로 변환
    # ------------------------------------------------------------------
    def _build_joint_trajectory_msg(self) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = self._joint_names

        for idx, (_, positions) in enumerate(self._trajectory):
            pt = JointTrajectoryPoint()
            pt.positions = positions
            t = idx * self._dt
            pt.time_from_start = self._to_duration(t)
            jt.points.append(pt)

        return jt

    # ------------------------------------------------------------------
    # CSV Trajectory 실행
    # ------------------------------------------------------------------
    def _send_csv_trajectory_goal(self) -> None:
        traj_msg = self._build_joint_trajectory_msg()
        if not traj_msg.points:
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.get_logger().info(f"CSV Trajectory 실행 시작 (Points: {len(traj_msg.points)})")

        if not self._fjt_client.wait_for_server(timeout_sec=10.0):
            return

        send_future = self._fjt_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("CSV Trajectory Goal 거부됨")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("CSV 실행 완료.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsvMoveitPlayer()
    try:
        # 1) MoveIt2로 현재→첫 포인트 플래닝 (장애물 회피)
        if not node._plan_and_execute_to_first_point():
            node.get_logger().error("초기 이동 실패. 중단합니다.")
            return

        # 2) CSV trajectory 실행
        node._send_csv_trajectory_goal()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()