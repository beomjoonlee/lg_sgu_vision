#!/usr/bin/env python3
import csv
from pathlib import Path
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
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
)


class CsvMoveitPlayer(Node):
    """
    1) MoveIt2의 /plan_kinematic_path 서비스를 이용해서
       현재 joint 상태 -> CSV 첫 joint 상태까지 모션 플래닝
       → 플래닝 결과 trajectory를 0.01초 간격으로 재타이밍해서 실행
    2) 그 이후 CSV joint trajectory 전체를
       → 0.01초 간격으로 재타이밍해서 FollowJointTrajectory로 실행
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
        self.declare_parameter(
            "csv_path",
            str(default_csv_path),
        )
        self.declare_parameter(
            "group_name",
            "ur_manipulator",  # ur_moveit_config 에서 사용하는 planning group 이름
        )
        self.declare_parameter(
            "controller_name",
            "scaled_joint_trajectory_controller",  # FollowJointTrajectory 컨트롤러 이름
        )
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

        self._csv_path = Path(self.get_parameter("csv_path").value)
        self._group_name: str = self.get_parameter("group_name").value
        self._controller_name: str = self.get_parameter("controller_name").value
        self._joint_names: List[str] = list(self.get_parameter("joint_names").value)

        self.get_logger().info(f"[trajectory_player] CSV path: {self._csv_path}")
        self.get_logger().info(f"[trajectory_player] MoveIt group: {self._group_name}")
        self.get_logger().info(f"[trajectory_player] Controller: {self._controller_name}")
        self.get_logger().info(f"[trajectory_player] Joint names: {self._joint_names}")
        self.get_logger().info(f"[trajectory_player] dt between waypoints: {self._dt} s")

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
    # JointState 콜백
    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg: JointState) -> None:
        self._current_joint_state = msg

    # ------------------------------------------------------------------
    # CSV 로드: (time, [joint_positions]) 리스트 반환
    #   ※ time 컬럼은 읽지만, 실제 실행에서는 무시하고 0.01초 간격으로 재타이밍함
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
                # joint 순서는 self._joint_names 기준으로 읽어온다.
                positions = [float(row[name]) for name in self._joint_names]
                traj.append((t, positions))

        self.get_logger().info(f"CSV에서 {len(traj)} 개의 trajectory point를 읽었습니다.")
        return traj

    # ------------------------------------------------------------------
    # float 초 → Duration 변환 (sec + nanosec)
    # ------------------------------------------------------------------
    @staticmethod
    def _to_duration(t: float) -> Duration:
        """예: t=0.01 → sec=0, nanosec=10000000"""
        sec = int(t)
        nanosec = int(round((t - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    # ------------------------------------------------------------------
    # MoveIt2: 현재 상태 → CSV 첫 포인트까지 플래닝 (GetMotionPlan)
    # 그리고 그 trajectory를 dt 간격으로 재타이밍해서 FJT로 실행
    # ------------------------------------------------------------------
    def _plan_and_execute_to_first_point(self) -> bool:
        first_time, first_positions = self._trajectory[0]
        self.get_logger().info(
            f"[trajectory_player] MoveIt2 (plan_kinematic_path) 로 "
            f"현재 상태에서 첫 CSV 포인트까지 플래닝 (t={first_time})"
        )

        # 1) joint_states 수신 대기
        self.get_logger().info("[trajectory_player] joint_states 수신 대기 중...")
        wait_cnt = 0
        while rclpy.ok() and self._current_joint_state is None and wait_cnt < 100:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_cnt += 1

        if self._current_joint_state is None:
            self.get_logger().error("joint_states를 받지 못했습니다. (timeout)")
            return False

        self.get_logger().info(
            f"[trajectory_player] joint_states 수신 완료. name={list(self._current_joint_state.name)}"
        )

        # 2) motion plan service 준비 대기
        self.get_logger().info("[trajectory_player] /plan_kinematic_path 서비스 대기 중...")
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

        # 시작 상태: 현재 joint_states
        start_state = RobotState()
        start_state.joint_state = self._current_joint_state
        mreq.start_state = start_state

        # 목표 joint constraint
        constraints = Constraints()
        for name, pos in zip(self._joint_names, first_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 1e-3  # 0.001 rad
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        mreq.goal_constraints.append(constraints)

        self.get_logger().info(
            "[trajectory_player] /plan_kinematic_path 서비스 호출 (joint goal)..."
        )
        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error("plan_kinematic_path 서비스 호출 실패")
            return False

        res = future.result()
        error_code = res.motion_plan_response.error_code.val

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveIt 플래닝 실패. error_code={error_code}"
            )
            return False

        traj = res.motion_plan_response.trajectory.joint_trajectory
        n_pts = len(traj.points)
        self.get_logger().info(
            f"[trajectory_player] MoveIt 플래닝 성공. planned trajectory points={n_pts}"
        )

        if n_pts == 0:
            self.get_logger().error("플래닝된 trajectory에 point가 없습니다.")
            return False

        # # === 여기서 첫 구간 trajectory를 dt 간격으로 재타이밍 ===
        # self.get_logger().info(
        #     f"[trajectory_player] 첫 구간 trajectory time_from_start를 "
        #     f"{self._dt}초 간격으로 재설정합니다."
        # )
        # for i, p in enumerate(traj.points):
        #     # 첫 점: t=0.0, 그 다음: dt, 2*dt, ...
        #     t = i * self._dt
        #     p.time_from_start = self._to_duration(t)

        # # 4) 플래닝된 trajectory를 FJT 컨트롤러로 실행
        # self.get_logger().info(
        #     "[trajectory_player] 첫 구간 trajectory를 FJT로 실행합니다."
        # )

        # if not self._fjt_client.wait_for_server(timeout_sec=10.0):
        #     self.get_logger().error(
        #         "FollowJointTrajectory action server를 찾지 못했습니다."
        #     )
        #     return False

        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = traj

        send_future = self._fjt_client.send_goal_async(fjt_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("첫 구간 trajectory goal이 거부되었습니다.")
            return False

        self.get_logger().info(
            "[trajectory_player] 첫 구간 trajectory goal 수락됨. 결과 대기 중..."
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f"[trajectory_player] 첫 구간 trajectory 실행 완료. "
            f"error_code={result.error_code}"
        )

        return True

    # ------------------------------------------------------------------
    # CSV 전체를 JointTrajectory 메시지로 변환
    #   ※ CSV의 time 컬럼은 무시하고, index * dt 로 time_from_start를 세팅
    # ------------------------------------------------------------------
    def _build_joint_trajectory_msg(self) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = self._joint_names

        for idx, (_, positions) in enumerate(self._trajectory):
            pt = JointTrajectoryPoint()
            pt.positions = positions
            t = idx * self._dt  # 0.0, 0.01, 0.02, ...
            pt.time_from_start = self._to_duration(t)
            jt.points.append(pt)

        return jt

    # ------------------------------------------------------------------
    # CSV Trajectory → FollowJointTrajectory 액션으로 전송 (동기)
    # ------------------------------------------------------------------
    def _send_csv_trajectory_goal(self) -> None:
        traj_msg = self._build_joint_trajectory_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.get_logger().info(
            f"[trajectory_player] CSV trajectory를 FollowJointTrajectory 액션으로 전송합니다. "
            f"points={len(traj_msg.points)}, dt={self._dt}"
        )

        self.get_logger().info("[trajectory_player] FJT action server 대기 중...")
        if not self._fjt_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "FollowJointTrajectory action server를 찾지 못했습니다."
            )
            return

        send_future = self._fjt_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("CSV trajectory goal이 거부되었습니다.")
            return

        self.get_logger().info(
            "[trajectory_player] CSV trajectory goal 수락됨. 결과 대기 중..."
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f"[trajectory_player] CSV trajectory 실행 완료. "
            f"error_code={result.error_code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsvMoveitPlayer()
    try:
        # 1) MoveIt2로 현재→첫 포인트 플래닝 + 실행 (0.01초 간격)
        if not node._plan_and_execute_to_first_point():
            node.get_logger().error("초기 MoveIt 플래닝/실행 실패. CSV 재생 중단.")
            return

        # 2) CSV trajectory 실행 (0.01초 간격)
        node._send_csv_trajectory_goal()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
