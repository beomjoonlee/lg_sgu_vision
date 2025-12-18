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


class TopicMoveitPlayer(Node):
    """
    1) MoveIt2의 /plan_kinematic_path 서비스를 이용해서
       현재 joint 상태 -> (받은 JointTrajectory의 첫 포인트) 까지 모션 플래닝
       → 플래닝 결과 trajectory를 그대로 FJT로 실행
    2) 그 이후, 받은 JointTrajectory 전체를
       → FollowJointTrajectory 액션으로 실행

    ※ 더 이상 CSV 파일은 사용하지 않고,
       외부에서 퍼블리시되는 JointTrajectory 토픽을 입력으로 사용합니다.
    """

    def __init__(self) -> None:
        super().__init__("topic_moveit_player")

        # ---- (기존) dt 파라미터는 남겨두지만, 현재 코드는 사용하지 않음 ----
        self._dt = 0.01

        # --- Parameters ---
        # CSV는 더 이상 사용하지 않지만, launch 파일 호환성을 위해 파라미터 선언만 남겨둠
        self.declare_parameter(
            "csv_path",
            "trajectory.csv",
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
        # 새로 추가: 외부에서 퍼블리시되는 JointTrajectory 토픽 이름
        self.declare_parameter(
            "input_trajectory_topic",
            "/joint_trajectory_from_csv",  # 4_publish_trajectory.py가 퍼블리시하도록 맞춰주면 됨
        )

        # 기존 파라미터 값들
        # self._csv_path = Path(self.get_parameter("csv_path").value)  # 더 이상 사용하지 않음
        self._group_name: str = self.get_parameter("group_name").value
        self._controller_name: str = self.get_parameter("controller_name").value
        self._joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self._input_topic: str = self.get_parameter("input_trajectory_topic").value

        self.get_logger().info(f"[trajectory_player] MoveIt group: {self._group_name}")
        self.get_logger().info(f"[trajectory_player] Controller: {self._controller_name}")
        self.get_logger().info(f"[trajectory_player] Joint names: {self._joint_names}")
        self.get_logger().info(
            f"[trajectory_player] Input trajectory topic: {self._input_topic}"
        )

        # --- 더 이상 CSV는 사용하지 않으므로 _trajectory 대신 JointTrajectory를 직접 보관 ---
        self._received_traj: Optional[JointTrajectory] = None

        # --- 현재 joint state 구독 ---
        self._current_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        # --- JointTrajectory 입력 구독 (4_publish_trajectory.py가 퍼블리시하는 토픽) ---
        self._traj_sub = self.create_subscription(
            JointTrajectory,
            self._input_topic,
            self._traj_callback,
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
    # JointTrajectory 콜백 (4_publish_trajectory.py가 보내는 경로 수신)
    # ------------------------------------------------------------------
    def _traj_callback(self, msg: JointTrajectory) -> None:
        # 처음 한 번만 저장 (원하면 마지막 것만 사용하도록 바꿔도 됨)
        if self._received_traj is None:
            self._received_traj = msg
            self.get_logger().info(
                f"[trajectory_player] JointTrajectory 수신: "
                f"joint_names={list(msg.joint_names)}, points={len(msg.points)}"
            )
        else:
            # 필요하면 최신 경로로 덮어쓰고 싶을 때 여기를 수정
            pass

    # ------------------------------------------------------------------
    # (이전 코드에서 사용하던 CSV 로더는 더 이상 쓰지 않지만, 남겨만 둠)
    # ------------------------------------------------------------------
    def _load_csv(self, path: Path) -> List[Tuple[float, List[float]]]:
        self.get_logger().warn(
            "[trajectory_player] _load_csv는 현재 사용되지 않습니다. "
            "JointTrajectory 토픽 입력 기반으로 동작합니다."
        )
        return []

    # ------------------------------------------------------------------
    # float 초 → Duration 변환 (sec + nanosec) – 필요 시 fallback 등에 사용 가능
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
    # MoveIt2: 현재 상태 → "받은 trajectory의 첫 포인트"까지 플래닝 + 실행
    # ------------------------------------------------------------------
    def _plan_and_execute_to_first_point(self) -> bool:
        # 0) joint_states & JointTrajectory 둘 다 들어올 때까지 대기
        self.get_logger().info(
            "[trajectory_player] joint_states 와 입력 JointTrajectory 수신 대기 중..."
        )
        self.get_logger().info(
            "[trajectory_player] joint_states + JointTrajectory가 들어올 때까지 무한 대기..."
        )
        while rclpy.ok() and (self._current_joint_state is None or self._received_traj is None):
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("[trajectory_player] 두 메시지 모두 수신됨. 실행 시작!")


        if self._current_joint_state is None:
            self.get_logger().error("joint_states를 받지 못했습니다. (timeout)")
            return False

        if self._received_traj is None:
            self.get_logger().error("입력 JointTrajectory를 받지 못했습니다. (timeout)")
            return False

        traj_msg = self._received_traj
        if not traj_msg.points:
            self.get_logger().error("입력 JointTrajectory에 point가 없습니다.")
            return False

        self.get_logger().info(
            f"[trajectory_player] joint_states / JointTrajectory 수신 완료. "
            f"traj points={len(traj_msg.points)}"
        )

        # 1) motion plan service 준비 대기
        self.get_logger().info("[trajectory_player] /plan_kinematic_path 서비스 대기 중...")
        if not self._plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/plan_kinematic_path 서비스를 찾지 못했습니다.")
            return False

        # 2) 목표 joint positions: 입력 JointTrajectory의 첫 포인트를 기준으로 사용
        first_point = traj_msg.points[0]
        # joint_names 매핑: self._joint_names 순서대로 값을 뽑아온다.
        name_to_index = {name: i for i, name in enumerate(traj_msg.joint_names)}
        first_positions: List[float] = []
        try:
            for jn in self._joint_names:
                idx = name_to_index[jn]
                first_positions.append(first_point.positions[idx])
        except KeyError as e:
            self.get_logger().error(
                f"입력 JointTrajectory의 joint_names에 {e} 가 없습니다. "
                f"traj joint_names={traj_msg.joint_names}, "
                f"expected={self._joint_names}"
            )
            return False

        self.get_logger().info(
            f"[trajectory_player] MoveIt2 (plan_kinematic_path) 로 "
            f"현재 상태에서 입력 trajectory 첫 포인트까지 플래닝"
        )

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

        # 4) 플래닝된 trajectory를 그대로 FJT 컨트롤러로 실행
        self.get_logger().info(
            "[trajectory_player] 첫 구간 trajectory를 FJT로 실행합니다."
        )

        if not self._fjt_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "FollowJointTrajectory action server를 찾지 못했습니다."
            )
            return False

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
    # 입력 JointTrajectory를 self._joint_names 순서로 재정렬해서 반환
    #   - time_from_start 은 입력 메시지 그대로 사용 (4_publish_trajectory.py의 dt)
    # ------------------------------------------------------------------
    def _build_joint_trajectory_msg(self) -> JointTrajectory:
        if self._received_traj is None:
            self.get_logger().error(
                "입력 JointTrajectory가 없습니다. _build_joint_trajectory_msg 실패."
            )
            return JointTrajectory()

        src = self._received_traj
        jt = JointTrajectory()
        jt.joint_names = self._joint_names

        # joint name → index 매핑
        name_to_index = {name: i for i, name in enumerate(src.joint_names)}

        for src_pt in src.points:
            pt = JointTrajectoryPoint()

            # positions 재정렬
            positions: List[float] = []
            try:
                for jn in self._joint_names:
                    idx = name_to_index[jn]
                    positions.append(src_pt.positions[idx])
            except KeyError as e:
                self.get_logger().error(
                    f"입력 JointTrajectory의 joint_names에 {e} 가 없습니다. "
                    f"traj joint_names={src.joint_names}, expected={self._joint_names}"
                )
                break

            pt.positions = positions

            # velocities/accelerations 등이 있으면 그대로 복사 (길이 맞을 때만)
            if src_pt.velocities:
                pt.velocities = [
                    src_pt.velocities[name_to_index[jn]] for jn in self._joint_names
                ]
            if src_pt.accelerations:
                pt.accelerations = [
                    src_pt.accelerations[name_to_index[jn]] for jn in self._joint_names
                ]
            if src_pt.effort:
                pt.effort = [
                    src_pt.effort[name_to_index[jn]] for jn in self._joint_names
                ]

            # time_from_start 는 입력 trajectory 그대로 사용
            pt.time_from_start = src_pt.time_from_start

            jt.points.append(pt)

        return jt

    # ------------------------------------------------------------------
    # 입력 Trajectory → FollowJointTrajectory 액션으로 전송 (동기)
    # ------------------------------------------------------------------
    def _send_csv_trajectory_goal(self) -> None:
        traj_msg = self._build_joint_trajectory_msg()

        if not traj_msg.points:
            self.get_logger().error(
                "전송할 trajectory가 비어 있습니다. (입력 JointTrajectory 없음?)"
            )
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.get_logger().info(
            f"[trajectory_player] 입력 JointTrajectory를 FollowJointTrajectory "
            f"액션으로 전송합니다. points={len(traj_msg.points)}"
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
            self.get_logger().error("입력 JointTrajectory goal이 거부되었습니다.")
            return

        self.get_logger().info(
            "[trajectory_player] 입력 JointTrajectory goal 수락됨. 결과 대기 중..."
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f"[trajectory_player] 입력 JointTrajectory 실행 완료. "
            f"error_code={result.error_code}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicMoveitPlayer()
    try:
        # 1) MoveIt2로 현재→첫 포인트 플래닝 + 실행
        if not node._plan_and_execute_to_first_point():
            node.get_logger().error("초기 MoveIt 플래닝/실행 실패. 입력 trajectory 재생 중단.")
            return

        # 2) 입력 JointTrajectory 전체 실행
        node._send_csv_trajectory_goal()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
