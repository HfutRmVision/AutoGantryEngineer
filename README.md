# AutoGantryEngineer

Autonomous dual-arm gantry robot manipulation system for the **RoboMaster** engineering challenge. Built on ROS 2 Humble with MoveIt Task Constructor (MTC) motion planning and BehaviorTree.CPP task orchestration.

## Overview

The gantry engineer robot is a **dual-arm gantry manipulator** with a shared horizontal X-rail and two mirror-symmetric 6-DOF arms, each ending in a two-finger gripper. The system performs autonomous cylinder manipulation sequences — alignment, grasping, pushing, and multi-axis rotation — through a behavior-tree-orchestrated pipeline of MTC motion planning stages.

### Hardware

- **Base**: Prismatic X-rail gantry (range -0.4 m ~ +0.2 m)
- **Each arm**: 6 revolute joints + 1 prismatic gripper joint (mimic fingers)
- **Total DOF**: 13 controllable joints + 2 grippers
- **End effector**: Two-finger gripper with 20 mm range
- **Lower-level MCU**: STM32 communicating over UART serial (115200 baud, 8N1)
- **Sensors**: Joint position/velocity feedback from MCU

## Architecture

```
┌─────────────────────────┐
│  BehaviorTree.CPP       │  Task orchestration
│  bt_orchestrator_node   │  Phase sequencing
└───────────┬─────────────┘
            │ tick (100 ms)
┌───────────▼─────────────┐
│  Phase Nodes (MTC)      │  Phase0: Align & Load
│  FuzzyPoseGenerator     │  Phase1: Push
│  CircularPathGenerator  │  Phase2: Circular Rotation (-90°)
│  rankSolutionsByJoint   │  Phase3: Y-axis Rotation (+45°)
│  MotionCost             │
└───────────┬─────────────┘
            │ /execute_task_solution
┌───────────▼─────────────┐
│  MoveIt 2 + MTC         │  TRAC-IK kinematics
│  move_group             │  OMPL / Pilz planners
└───────────┬─────────────┘
            │ joint_trajectory_controller
┌───────────▼─────────────┐
│  ros2_control           │  JointTrajectoryController
│  controller_manager     │  + JointStateBroadcaster
└───────────┬─────────────┘
            │ write() / read()
┌───────────▼─────────────┐
│  GantryRobotHardware     │  ros2_control SystemInterface
│  Interface              │  6 joints (position cmd/state)
└───────────┬─────────────┘
            │ EngineerProtocol
┌───────────▼─────────────┐
│  rm_serial_driver       │  FixedPacket<64> + CRC-16
│  UartTransporter        │  /dev/ttyACM0 @ 115200
└───────────┬─────────────┘
            │ UART
┌───────────▼─────────────┐
│  STM32 MCU              │  Low-level motor control
└─────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `rm_interfaces` | Custom ROS 2 messages (`GimbalCmd`, `SerialReceiveData`) and services (`SetMode`) |
| `rm_serial_driver` | UART serial communication library: packet framing (0xFF/0x0D), CRC-16, protocol abstraction, transport interface |
| `gantry_robot_hardware_interface` | ros2_control `SystemInterface` plugin — bridges JointTrajectoryController to MCU via serial protocol |
| `controller` | MoveIt Task Constructor (MTC) phase nodes: fuzzy pose generation, circular path planning, joint-motion cost ranking |
| `orchestrator` | BehaviorTree.CPP v4 nodes — executes the 4-phase manipulation sequence as a BT |
| `motion_planning_test` | Test nodes: end-effector alignment, stand random pose, target scene publisher |
| `model` / `models` | Robot URDF/Xacro descriptions and STL meshes (dual-arm + single-arm + stand) |
| `gantry_robot_moveit_config` | MoveIt 2 configuration for real hardware (TRAC-IK, joint limits, controllers) |
| `gantry_robot_moveit_config_sim` | MoveIt 2 configuration for simulation |
| `gantry_engineer_moveit_config` | MoveIt 2 configuration for the dual-arm "engineer" variant |
| `bringup` | Launch files and ros2_control YAML configuration |

## Dependency Graph

```
rm_interfaces
    ▲
rm_serial_driver ──────────────────────┐
    ▲                                   │
gantry_robot_hardware_interface         │
    ▲                                   │
controller ─────────────────────────────┤
    ▲                                   │
orchestrator ───────────────────────────┤
    ▲                                   │
motion_planning_test ───────────────────┘
    ▲
model / models
    ▲
gantry_*_moveit_config ─── bringup
```

## Manipulation Pipeline

The Behavior Tree (`src/orchestrator/config/gantry_task.xml`) executes four phases in sequence:

### Phase 0 — Align & Load
- Waits for `cylinder_target_frame` and `link7_1` TF transforms
- Moves from current state to preload position (sampling planner, +3 cm Z offset)
- **Fuzzy Pose Generation**: Samples 30 noisy IK targets around the nominal pose (uniform noise within position/orientation tolerance, ±60° yaw range)
- Moves down to load position (Cartesian planner, -5 cm Z) and attaches the cylinder object

### Phase 1 — Push
- Attaches the cylinder to the end effector in the planning scene
- Pushes the cylinder 0.1 m along its local Y-axis
- Uses sampling planner at reduced velocity (0.1×) for safe motion
- **Fuzzy Pose Generation**: 50 samples around the push goal pose

### Phase 2 — Circular Rotation (-90°)
- Detaches the cylinder, repositions, re-grasps
- Generates an arc path using **Rodrigues' rotation formula**
- Pivot axis: offset (y=108 mm, z=-169 mm) from cylinder origin
- 12 Cartesian waypoints over -90° rotation
- **Fallback**: On failure, moves to last plannable waypoint, then recovers to `home_gr`
- **Joint-motion cost ranking**: Heavily penalizes X-rail movement (65% weight) vs. arm joints (35% weight)

### Phase 3 — Y-axis Rotation (+45°)
- Detaches, repositions, re-grasps
- Two alternative planning strategies attempted:
  - **Strategy A**: Pilz CIRC circular motion planner with interim sphere constraint (5 mm radius)
  - **Strategy B**: 24-step Cartesian path (1.5 mm step size)
- Rotation axis: local Y-axis of derived target frame, Z-offset -54 mm
- Fallback to `home_gr` on failure

### Safety
- `recover_home_node` monitors end-effector Z-height at 5 Hz
- If Z drops below `recover_z_threshold` (default 45 mm), auto-plans and executes motion to `home_gr`

## Serial Protocol

**Physical**: UART, 115200 baud, 8N1, `/dev/ttyACM0`

**Packet** (`FixedPacket<64>`):

```
┌──────┬──────┬───────────┬──────────────┬───────┬──────┐
│ 0xFF │ mode │ positions │  velocities  │ CRC16 │ 0x0D │
│ (1B) │ (1B) │  (6×4B)   │   (6×4B)     │ (2B)  │ (1B) │
└──────┴──────┴───────────┴──────────────┴───────┴──────┘
```

- **mode**: `3` = ros2_control active
- **positions/velocities**: float32, 6 joints
- **CRC-16**: CCITT polynomial
- Coordinate transforms between MCU (mm, raw) and ROS (m, SI) are handled in `EngineerProtocol`

**Auto-send**: `EngineerProtocol` subscribes to `/display_planned_path` and streams trajectory points over serial with 50 ms spacing.

## Getting Started

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 + MoveIt Task Constructor
- ros2_control + ros2_controllers
- BehaviorTree.CPP v4
- TRAC-IK kinematics plugin
- Pilz industrial motion planner (optional, for Phase 3)

### Build

```bash
cd ~/ws_sim
colcon build --symlink-install
source install/setup.bash
```

### Run

**1. Real Hardware (with MCU connected)**

```bash
# Launch MoveIt + ros2_control
ros2 launch gantry_robot_moveit_config demo.launch.py

# Launch the Behavior Tree orchestrator
ros2 launch orchestrator bt_orchestrator.launch.py

# (Optional) Publish a target scene object
ros2 launch motion_planning_test target_scene_publisher.launch.py
```

**2. Simulation (no hardware)**

```bash
# Launch MoveIt in simulation mode
ros2 launch gantry_robot_moveit_config_sim demo.launch.py

# Launch the BT orchestrator (with sim config)
ros2 launch orchestrator bt_orchestrator.launch.py
```

**3. Running with the dual-arm "engineer" config**

```bash
ros2 launch gantry_engineer_moveit_config demo.launch.py
ros2 launch orchestrator bt_orchestrator.launch.py
```

### Test Nodes

```bash
# End-effector alignment test (gantry → stand)
ros2 run motion_planning_test ee_alignment_test

# Stand random pose test
ros2 run motion_planning_test stand_random_pose

# Serial driver test
ros2 run orchestrator serial_test_node
```

## Configuration

Key parameters to tune:

| Parameter | File | Description |
|-----------|------|-------------|
| Serial port | `gantry_robot_hardware_interface` | `/dev/ttyACM0` — change if using different port |
| Joint limits | `gantry_robot_moveit_config/config/joint_limits.yaml` | Velocity/acceleration limits |
| Recovery Z threshold | `recover_home_node` | Default 0.045 m — triggers auto-recovery |
| Fuzzy sample count | Phase nodes (0–3) | 20–50 samples — more = better plans, slower |
| MTC planning attempts | `MTC::doTask()` | Default 5 — increases robustness |
| Vel scaling | Phase 1 connect stage | 0.1 — safety slowdown during pushing |
| Rail motion cost weight | `rankSolutionsByJointMotionCost()` | 0.65 — higher = prefer arm motion over rail |

## Related Repositories

This project is part of the HfutRmVision RoboMaster ecosystem and uses components adapted from:
- RoboMaster serial communication protocol (FYT Vision Group template)
- MoveIt Task Constructor framework
- BehaviorTree.CPP v4

## License

This project is developed by the HfutRmVision team for RoboMaster competition use.
