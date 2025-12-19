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
    CollisionObject,  # 장애물 추가를 위해 필요
)


class TopicMoveitPlayer(Node):
    """
    1) MoveIt2의 /plan_kinematic_path 서비스를 이용해서
       현재 joint 상태 -> (받은 JointTrajectory의 첫 포인트) 까지 모션 플래닝
       → 플래닝 결과 trajectory를 그대로 FJT로 실행
    2) 그 이후, 받은 JointTrajectory 전체를
       → FollowJointTrajectory 액션으로 실행
    3) 시작 시 정의된 장애물(Table, Walls)을 MoveIt Planning Scene에 추가
    """

    def __init__(self) -> None:
        super().__init__("topic_moveit_player")

        self._dt = 0.01

        # --- Parameters ---
        self.declare_parameter("csv_path", "trajectory.csv")
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
        self.declare_parameter("input_trajectory_topic", "/joint_trajectory_from_csv")
        # 장애물이 배치될 기준 프레임 (보통 'base_link' 또는 'world')
        self.declare_parameter("planning_frame", "base_link")

        self._group_name: str = self.get_parameter("group_name").value
        self._controller_name: str = self.get_parameter("controller_name").value
        self._joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self._input_topic: str = self.get_parameter("input_trajectory_topic").value
        self._planning_frame: str = self.get_parameter("planning_frame").value

        self.get_logger().info(f"[trajectory_player] MoveIt group: {self._group_name}")
        self.get_logger().info(f"[trajectory_player] Controller: {self._controller_name}")
        self.get_logger().info(f"[trajectory_player] Planning Frame: {self._planning_frame}")

        self._received_traj: Optional[JointTrajectory] = None

        # --- Collision Object Publisher (장애물 추가용) ---
        # MoveIt은 /collision_object 토픽을 구독하여 Planning Scene을 업데이트합니다.
        # Transient Local 설정을 통해 늦게 접속한 노드(MoveIt)도 메시지를 받을 수 있게 하면 좋습니다.
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", qos_profile
        )

        # --- 장애물 추가 실행 ---
        # Publisher가 생성된 직후 바로 보내면 씹힐 수 있으므로 약간의 딜레이 후 전송하거나,
        # 메인 로직 시작 전에 확실히 보냅니다.
        self.get_logger().info("[trajectory_player] 장애물(Collision Objects)을 추가합니다...")
        self._publish_obstacles()

        # --- 현재 joint state 구독 ---
        self._current_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        # --- JointTrajectory 입력 구독 ---
        self._traj_sub = self.create_subscription(
            JointTrajectory,
            self._input_topic,
            self._traj_callback,
            10,
        )

        # --- Action Client ---
        fjt_action_name = f"/{self._controller_name}/follow_joint_trajectory"
        self._fjt_client = ActionClient(self, FollowJointTrajectory, fjt_action_name)

        # --- Service Client ---
        self._plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")

    # ------------------------------------------------------------------
    # [NEW] 장애물 추가 메서드
    # ------------------------------------------------------------------
    def _publish_obstacles(self):
        # 1. 장애물 데이터 정의 (numpy array 사용)
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

        # 모든 장애물 리스트 합치기
        all_obstacles = [TABLE] + WALLS

        # [핵심 수정] MoveIt이 토픽을 구독할 때까지 대기
        self.get_logger().info("MoveIt(/collision_object) 연결 대기 중...")
        while self._collision_pub.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("MoveIt 연결 확인됨. 장애물 전송 시작.")


        for obs_data in all_obstacles:
            collision_object = CollisionObject()
            collision_object.header.frame_id = self._planning_frame
            collision_object.id = obs_data["name"]

            # 장애물 추가 (ADD)
            collision_object.operation = CollisionObject.ADD

            # 모양 정의 (Box)
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                float(obs_data["dimensions"][0]),
                float(obs_data["dimensions"][1]),
                float(obs_data["dimensions"][2]),
            ]

            # 위치 정의 (Pose)
            pose = Pose()
            pose.position.x = float(obs_data["position"][0])
            pose.position.y = float(obs_data["position"][1])
            pose.position.z = float(obs_data["position"][2])
            # 회전은 없다고 가정 (Identity quaternion)
            pose.orientation.w = 1.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)

            # Publish
            self._collision_pub.publish(collision_object)
            # 메시지가 확실히 전송되도록 아주 짧은 sleep (선택 사항)
            time.sleep(0.1)
        
        self.get_logger().info(f"[trajectory_player] 총 {len(all_obstacles)}개의 장애물 Publish 완료.")

    # ------------------------------------------------------------------
    # JointState 콜백
    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg: JointState) -> None:
        self._current_joint_state = msg

    # ------------------------------------------------------------------
    # JointTrajectory 콜백
    # ------------------------------------------------------------------
    def _traj_callback(self, msg: JointTrajectory) -> None:
        if self._received_traj is None:
            self._received_traj = msg
            self.get_logger().info(
                f"[trajectory_player] JointTrajectory 수신: "
                f"joint_names={list(msg.joint_names)}, points={len(msg.points)}"
            )

    # ------------------------------------------------------------------
    # MoveIt2: 현재 상태 → "받은 trajectory의 첫 포인트"까지 플래닝 + 실행
    # ------------------------------------------------------------------
    def _plan_and_execute_to_first_point(self) -> bool:
        self.get_logger().info(
            "[trajectory_player] joint_states + JointTrajectory가 들어올 때까지 대기..."
        )
        while rclpy.ok() and (self._current_joint_state is None or self._received_traj is None):
            # 대기 중에도 장애물 토픽이 잘 퍼블리시되도록 spin
            rclpy.spin_once(self, timeout_sec=0.1)
            # 혹시 모르니 장애물 정보 주기적으로 재전송 (MoveIt이 늦게 켜질 경우 대비)
            # self._publish_obstacles() 

        self.get_logger().info("[trajectory_player] 두 메시지 모두 수신됨. 실행 시작!")

        if self._current_joint_state is None or self._received_traj is None:
            return False

        traj_msg = self._received_traj
        if not traj_msg.points:
            self.get_logger().error("입력 JointTrajectory에 point가 없습니다.")
            return False

        # 1) motion plan service 준비 대기
        if not self._plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/plan_kinematic_path 서비스를 찾지 못했습니다.")
            return False

        # 2) 목표 joint positions 설정
        first_point = traj_msg.points[0]
        name_to_index = {name: i for i, name in enumerate(traj_msg.joint_names)}
        first_positions: List[float] = []
        try:
            for jn in self._joint_names:
                idx = name_to_index[jn]
                first_positions.append(first_point.positions[idx])
        except KeyError as e:
            self.get_logger().error(f"Joint matching error: {e}")
            return False

        # 3) GetMotionPlan 요청
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

        self.get_logger().info("[trajectory_player] 경로 플래닝 요청 (MoveIt)...")
        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result():
            self.get_logger().error("서비스 호출 실패")
            return False

        res = future.result()
        if res.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveIt 플래닝 실패 Code: {res.motion_plan_response.error_code.val}")
            return False

        traj = res.motion_plan_response.trajectory.joint_trajectory
        self.get_logger().info(f"[trajectory_player] 플래닝 성공. points={len(traj.points)}")

        # 4) 실행 (FollowJointTrajectory)
        if not self._fjt_client.wait_for_server(timeout_sec=10.0):
            return False

        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = traj
        
        send_future = self._fjt_client.send_goal_async(fjt_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        return True

    # ------------------------------------------------------------------
    # 입력 JointTrajectory 재정렬
    # ------------------------------------------------------------------
    def _build_joint_trajectory_msg(self) -> JointTrajectory:
        if self._received_traj is None:
            return JointTrajectory()

        src = self._received_traj
        jt = JointTrajectory()
        jt.joint_names = self._joint_names
        name_to_index = {name: i for i, name in enumerate(src.joint_names)}

        for src_pt in src.points:
            pt = JointTrajectoryPoint()
            positions: List[float] = []
            try:
                for jn in self._joint_names:
                    idx = name_to_index[jn]
                    positions.append(src_pt.positions[idx])
            except KeyError:
                break
            pt.positions = positions
            
            # velocity, acc, effort 등은 생략하거나 필요 시 복사
            # 여기서는 position과 time만 중요하다고 가정
            pt.time_from_start = src_pt.time_from_start
            jt.points.append(pt)

        return jt

    # ------------------------------------------------------------------
    # 입력 Trajectory 실행
    # ------------------------------------------------------------------
    def _send_csv_trajectory_goal(self) -> None:
        traj_msg = self._build_joint_trajectory_msg()
        if not traj_msg.points:
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg

        self.get_logger().info(f"[trajectory_player] CSV Trajectory 실행 시작 (pts={len(traj_msg.points)})")
        
        if not self._fjt_client.wait_for_server(timeout_sec=10.0):
            return

        send_future = self._fjt_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal 거부됨")
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info("[trajectory_player] 최종 실행 완료.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicMoveitPlayer()
    try:
        # 1) MoveIt2로 현재→첫 포인트 플래닝 (장애물 회피 포함) + 실행
        if not node._plan_and_execute_to_first_point():
            node.get_logger().error("초기 이동 실패. 종료합니다.")
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