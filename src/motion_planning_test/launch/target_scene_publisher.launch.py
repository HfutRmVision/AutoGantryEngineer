from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 参数声明
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'reference_frame',
            default_value='ipm',
            description='参考坐标系'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'target_name',
            default_value='cylinder_target',
            description='Target 名称'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'auto_publish',
            default_value='false',
            description='是否自动发布随机 target'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'publish_interval',
            default_value='5.0',
            description='自动发布间隔（秒）'
        )
    )
    
    # Target scene publisher 节点
    target_scene_publisher_node = Node(
        package="motion_planning_test",
        executable="target_scene_publisher",
        output="screen",
        parameters=[{
            'reference_frame': LaunchConfiguration('reference_frame'),
            'target_name': LaunchConfiguration('target_name'),
            'cylinder_radius': 0.018,
            'cylinder_length': 0.1,
            'auto_publish': LaunchConfiguration('auto_publish'),
            'publish_interval': LaunchConfiguration('publish_interval'),
            'use_evaluation_poses': False,
            # 位置范围 (mm)
            'x_min': -100.0,
            'x_max': 100.0,
            'y_min': -100.0,
            'y_max': 100.0,
            'z_min': -100.0,
            'z_max': 0.0,
            # 姿态范围 (度)
            'roll_min': -90.0,
            'roll_max': 0.0,
            'pitch_min': -90.0,
            'pitch_max': 90.0,
            'yaw_min': -45.0,
            'yaw_max': 45.0,
        }],
    )
    
    return LaunchDescription(declared_arguments + [
        target_scene_publisher_node,
    ])
