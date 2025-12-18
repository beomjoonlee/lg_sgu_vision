from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    csv_path = LaunchConfiguration('csv_path')
    group_name = LaunchConfiguration('group_name')
    controller_name = LaunchConfiguration('controller_name')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'csv_path',
                default_value='/home/lbj/vision_ws/src/trajectory_player/trajectory_player/trajectory.csv',
                description='CSV trajectory file path',
            ),
            DeclareLaunchArgument(
                'group_name',
                default_value='ur_manipulator',
                description='MoveIt planning group name',
            ),
            DeclareLaunchArgument(
                'controller_name',
                default_value='scaled_joint_trajectory_controller',
                description='FollowJointTrajectory controller name',
            ),
            Node(
                package='trajectory_player',
                executable='csv_moveit_player',
                name='csv_moveit_player',
                output='screen',
                parameters=[
                    {'csv_path': csv_path},
                    {'group_name': group_name},
                    {'controller_name': controller_name},
                ],
            ),
        ]
    )
