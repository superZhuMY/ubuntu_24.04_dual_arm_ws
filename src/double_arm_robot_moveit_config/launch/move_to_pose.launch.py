from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "double_arm_robot",
            package_name="double_arm_robot_moveit_config",
        )
        .to_moveit_configs()
    )

    move_to_pose_node = Node(
        package="double_arm_jaka_controller",
        executable="move_to_pose",
        name="move_to_pose_client",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_to_pose_node])
