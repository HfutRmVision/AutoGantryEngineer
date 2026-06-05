from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for hardware communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    enable_data_print_arg = DeclareLaunchArgument(
        'enable_data_print',
        default_value='false',
        description='Enable serial data print for debugging'
    )

    # Build MoveIt config with real hardware ros2_control
    moveit_config = MoveItConfigsBuilder(
        "gantry_robot", 
        package_name="gantry_robot_moveit_config"
    ).robot_description(
        file_path="config/gantry_robot.urdf.xacro",
        mappings={
            "ros2_control_hardware_type": "real",
            "serial_port": LaunchConfiguration('serial_port'), 
            "baud_rate": LaunchConfiguration('baud_rate'),     
            "enable_data_print": LaunchConfiguration('enable_data_print'), 
        }
    ).to_moveit_configs()
    
    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Ros2 control node for real hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("gantry_robot_moveit_config"),
        "config",
        "ros2_controllers_real.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers - 延迟启动以等待 ros2_control_node 完全初始化
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )

    gantry_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_robot_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )
    
    # 使用 TimerAction 延迟启动 spawner，确保 ros2_control_node 完全就绪
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner],
    )
    
    delayed_gantry_robot_controller_spawner = TimerAction(
        period=6.0,
        actions=[gantry_robot_controller_spawner],
    )

    # Move group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            {"use_sim_time": False},
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("gantry_robot_moveit_config"),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    recover_home_node = Node(
        package="controller",
        executable="recover_home_node",
        name="recover_home_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    delayed_recover_home_node = TimerAction(
        period=8.0,
        actions=[recover_home_node],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        enable_data_print_arg,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        delayed_joint_state_broadcaster_spawner,
        delayed_gantry_robot_controller_spawner,
        move_group_node,
        delayed_recover_home_node,
        rviz_node,
    ])
