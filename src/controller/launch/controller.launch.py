from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gantry_engineer", package_name="gantry_engineer_moveit_config").to_moveit_configs()

    # Controller node
    controller_node = Node(
        package="controller",
        executable="controller_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([controller_node])
