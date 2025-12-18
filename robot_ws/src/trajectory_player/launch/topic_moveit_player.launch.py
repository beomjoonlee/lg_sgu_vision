from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    group_name = LaunchConfiguration('group_name')
    controller_name = LaunchConfiguration('controller_name')
    input_topic = LaunchConfiguration('input_trajectory_topic')

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                'input_trajectory_topic',
                default_value='/joint_trajectory_from_csv',
                description='Input JointTrajectory topic name',
            ),
            Node(
                package='trajectory_player',
                executable='topic_moveit_player',
                name='topic_moveit_player',
                output='screen',
                parameters=[
                    {'group_name': group_name},
                    {'controller_name': controller_name},
                    {'input_trajectory_topic': input_topic},
                ],
            ),
        ]
    )
