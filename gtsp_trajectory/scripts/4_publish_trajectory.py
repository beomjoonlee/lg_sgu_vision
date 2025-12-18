#!/usr/bin/env python3
"""
ROS2 토픽으로 검사 궤적을 moveit으로 전송
로봇 컨트롤은 moveit을 사용하여 이루어짐

사용법:
    omni_python scripts/4_publish_trajectory.py \
        --object sample \
        --num_viewpoints 163

경로는 자동으로 생성됩니다:
- 입력: data/{object}/trajectory/{num_viewpoints}/trajectory.csv
"""

import csv
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Add parent directory to path for common module
sys.path.insert(0, str(Path(__file__).parent.parent))
from common import config


class JointTrajectoryPublisher(Node):
    def __init__(self, csv_path: str, dt: float = 0.01):
        super().__init__('joint_trajectory_publisher')

        self.dt = dt  # Time step between points (seconds)

        # UR20 joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # CSV column names mapping
        self.csv_joint_columns = [
            'ur20-shoulder_pan_joint',
            'ur20-shoulder_lift_joint',
            'ur20-elbow_joint',
            'ur20-wrist_1_joint',
            'ur20-wrist_2_joint',
            'ur20-wrist_3_joint'
        ]

        # Load trajectory from CSV
        self.trajectory_points = self.load_csv(csv_path)
        self.get_logger().info(f'Loaded {len(self.trajectory_points)} points (dt={dt}s, total={len(self.trajectory_points)*dt:.2f}s)')

        # Publisher
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_from_csv',
            10
        )

        # Publish once after a short delay to ensure connections are established
        self.timer = self.create_timer(5.0, self.publish_trajectory)
        self.published = False

    def load_csv(self, csv_path: str) -> list:
        """Load trajectory points from CSV file."""
        points = []
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                positions = [float(row[col]) for col in self.csv_joint_columns]
                points.append(positions)
        return points

    def publish_trajectory(self):
        """Publish the full trajectory."""
        if self.published:
            return

        # 🚀 subscriber 연결될 때까지 대기
        while self.pub.get_subscription_count() == 0:
            self.get_logger().info("아직 Subscriber 연결 안 됨. 기다리는 중...")
            rclpy.sleep(0.5)

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        # Convert all CSV points to JointTrajectoryPoints
        for i, positions in enumerate(self.trajectory_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 6

            # time_from_start = index * dt
            t = i * self.dt
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            msg.points.append(point)

        self.pub.publish(msg)
        self.get_logger().info(f'Published trajectory with {len(msg.points)} points')
        self.published = True

        # Keep node alive for a moment then shutdown
        self.create_timer(2.0, self.shutdown_node)

    def shutdown_node(self):
        self.get_logger().info('Trajectory published. Shutting down...')
        raise SystemExit


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description='Publish joint trajectory from CSV')
    parser.add_argument(
        '--object',
        type=str,
        required=True,
        help="Object name for auto-path generation (e.g., 'sample', 'glass')"
    )
    parser.add_argument(
        '--num_viewpoints',
        type=int,
        required=True,
        help='Number of viewpoints'
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.01,
        help='Time step between trajectory points in seconds (default: 0.01)'
    )
    parsed_args, remaining = parser.parse_known_args()

    # Auto-generate trajectory path
    csv_path = str(config.get_trajectory_path(parsed_args.object, parsed_args.num_viewpoints, "trajectory.csv"))

    print(f"Object: {parsed_args.object}")
    print(f"Num viewpoints: {parsed_args.num_viewpoints}")
    print(f"Trajectory path: {csv_path}")

    rclpy.init(args=remaining)
    node = JointTrajectoryPublisher(csv_path, parsed_args.dt)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
