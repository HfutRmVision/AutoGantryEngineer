from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "gantry_robot", package_name="gantry_engineer_moveit_config"
    ).to_moveit_configs()

    bt_orchestrator_node = Node(
        package="orchestrator",
        executable="bt_orchestrator_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.pilz_cartesian_limits,
            {"tick_rate_ms": 100},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        bt_orchestrator_node,
    ])
