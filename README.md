# AutoGantryEngineer

基于 ROS 2 Humble 的**双机械臂龙门机器人**自主操控系统，面向 **RMUL2026**。结合 MoveIt Task Constructor (MTC) 运动规划与 BehaviorTree.CPP 行为树编排，实现全自动圆柱体抓取、推移及多轴旋转等复杂操作。

## 项目概述

**双机械臂龙门式**操作平台，两个 6 自由度机械臂以镜像对称方式安装在共享的 X 轴水平导轨上，末端各装有一个两指夹爪。系统通过行为树编排 MTC 运动规划流水线，自主完成对齐、抓取、推移和多轴旋转等圆柱体操作任务序列。

### 硬件概览

- **底座**: X 轴直线导轨龙门（行程 -0.4 m ~ +0.2 m）
- **单臂**: 6 个旋转关节 + 1 个夹爪关节（联动手指）
- **总自由度**: 13 个可控关节 + 2 个夹爪
- **末端执行器**: 双指夹爪，开合范围 20 mm
- **下位机**: STM32，通过 UART 串口通信（115200 baud, 8N1）
- **传感器**: MCU 反馈的关节位置/速度

## 系统架构

```
┌─────────────────────────────┐
│  BehaviorTree.CPP           │  行为树任务编排
│  bt_orchestrator_node       │  阶段（Phase）调度
└─────────────┬───────────────┘
              │ tick (100 ms)
┌─────────────▼───────────────┐
│  Phase 节点 (MTC)            │  Phase0: 对齐与加载
│  FuzzyPoseGenerator         │  Phase1: 推移
│  CircularPathGenerator      │  Phase2: 圆弧旋转 (-90°)
│  rankSolutionsByJoint       │  Phase3: Y轴旋转 (+45°)
│  MotionCost                 │
└─────────────┬───────────────┘
              │ /execute_task_solution
┌─────────────▼───────────────┐
│  MoveIt 2 + MTC             │  TRAC-IK 运动学求解
│  move_group                 │  OMPL / Pilz 规划器
└─────────────┬───────────────┘
              │ joint_trajectory_controller
┌─────────────▼───────────────┐
│  ros2_control               │  JointTrajectoryController
│  controller_manager         │  + JointStateBroadcaster
└─────────────┬───────────────┘
              │ write() / read()
┌─────────────▼───────────────┐
│  GantryRobotHardware         │  ros2_control SystemInterface
│  Interface                  │  6 关节（位置指令/状态）
└─────────────┬───────────────┘
              │ EngineerProtocol
┌─────────────▼───────────────┐
│  rm_serial_driver           │  FixedPacket<64> + CRC-16
│  UartTransporter            │  /dev/ttyACM0 @ 115200
└─────────────┬───────────────┘
              │ UART
┌─────────────▼───────────────┐
│  STM32 MCU                  │  底层电机控制
└─────────────────────────────┘
```

## 功能包说明

| 功能包 | 说明 |
|--------|------|
| `rm_interfaces` | 自定义 ROS 2 消息（`GimbalCmd`, `SerialReceiveData`）与服务（`SetMode`） |
| `rm_serial_driver` | UART 串口通信库：数据帧封装（0xFF/0x0D）、CRC-16 校验、协议抽象层、传输接口 |
| `gantry_robot_hardware_interface` | ros2_control `SystemInterface` 插件 —— 桥接 JointTrajectoryController 与 MCU 串口协议 |
| `controller` | MoveIt Task Constructor (MTC) 阶段节点：模糊姿态生成、圆弧路径规划、关节运动代价排序 |
| `orchestrator` | BehaviorTree.CPP v4 节点 —— 以行为树形式执行四阶段操作序列 |
| `motion_planning_test` | 测试节点：末端对齐测试、台架随机姿态测试、目标场景发布器 |
| `model` / `models` | 机器人 URDF/Xacro 模型描述与 STL 网格（双臂 + 单臂 + 台架） |
| `gantry_robot_moveit_config` | 真机 MoveIt 2 配置（TRAC-IK、关节限位、控制器） |
| `gantry_robot_moveit_config_sim` | 仿真 MoveIt 2 配置 |
| `gantry_engineer_moveit_config` | 双臂 "Engineer" 变体 MoveIt 2 配置 |
| `bringup` | Launch 启动文件与 ros2_control YAML 配置 |

## 依赖关系图

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

## 目录结构

```
ws_sim/
├── src/
│   ├── rm_interfaces/                       # 自定义消息与服务
│   │   ├── msg/GimbalCmd.msg
│   │   ├── msg/SerialReceiveData.msg
│   │   └── srv/SetMode.srv
│   ├── rm_serial_driver/                    # 串口通信驱动
│   │   ├── include/rm_serial_driver/
│   │   │   ├── fixed_packet.hpp             # 定长数据帧 (0xFF/0x0D)
│   │   │   ├── fixed_packet_tool.hpp        # CRC 校验、组帧解帧
│   │   │   ├── transporter_interface.hpp    # 传输层抽象接口
│   │   │   ├── uart_transporter.hpp         # Linux termios UART 实现
│   │   │   ├── protocol.hpp                 # 协议抽象层
│   │   │   ├── protocol/engineer_protocol.hpp  # 工程机器人具体协议
│   │   │   └── protocol_factory.hpp
│   │   └── src/
│   ├── gantry_robot_hardware_interface/     # ros2_control 硬件接口
│   │   ├── include/.../gantry_robot_hardware_interface.hpp
│   │   └── src/gantry_robot_hardware_interface.cpp
│   ├── controller/                          # MTC 运动规划
│   │   ├── include/controller/
│   │   │   ├── mtc.hpp                      # MTC 基类（代价排序等）
│   │   │   ├── fuzzy_pose_generator.hpp     # 模糊姿态生成器
│   │   │   ├── circular_path_generator.hpp  # 圆弧路径生成
│   │   │   ├── phase0_node.hpp              # 对齐与加载
│   │   │   ├── phase1_node.hpp              # 推移
│   │   │   ├── phase2_node.hpp              # 圆弧旋转 (-90°)
│   │   │   ├── phase3_node.hpp              # Y轴旋转 (+45°)
│   │   │   ├── phase_collection_node.hpp    # 组合阶段
│   │   │   └── phase_test_node.hpp
│   │   └── src/
│   │       ├── controller_node.cpp          # 主入口
│   │       └── recover_home_node.cpp        # 安全监控/自动归位
│   ├── orchestrator/                        # 行为树编排
│   │   ├── include/orchestrator/
│   │   │   ├── orchestrator.hpp             # 编排器（阶段管理）
│   │   │   ├── bt_mtc_nodes.hpp             # BT MTC 动作节点
│   │   │   └── phase_interface.hpp          # 阶段状态机
│   │   ├── config/gantry_task.xml           # 行为树 XML 定义
│   │   └── src/bt_orchestrator_node.cpp     # BT 编排主节点
│   ├── motion_planning_test/               # 测试与工具
│   │   ├── src/ee_alignment_test.cpp        # 末端对齐测试
│   │   ├── src/stand_random_pose.cpp        # 台架随机姿态
│   │   └── src/target_scene_publisher.cpp   # 目标场景圆柱体发布
│   ├── models/                              # 机器人模型（单臂 + 台架）
│   │   ├── urdf/gantry_robot.xacro
│   │   ├── urdf/stand.xacro
│   │   └── meshes/                          # STL 网格文件
│   ├── model/                               # 机器人模型（双臂）
│   │   ├── urdf/gantry_engineer.urdf
│   │   └── meshes/                          # 详细 STL 网格
│   ├── bringup/                             # 启动与控制器配置
│   ├── gantry_robot_moveit_config/          # 真机 MoveIt 配置
│   ├── gantry_robot_moveit_config_sim/      # 仿真 MoveIt 配置
│   └── gantry_engineer_moveit_config/       # 双臂 MoveIt 配置
└── README.md
```

## ROS 2 接口

### 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 当前关节状态 |
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | 订阅 | 轨迹可视化（自动串口转发） |
| `/orchestrator/phase` | `std_msgs/Int32` | 发布 | 当前阶段号 |
| `/orchestrator/states` | `std_msgs/Int32` | 订阅 | 状态累积信号 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | 启停控制器 |
| `/publish_target_3_1` | `std_srvs/Trigger` | 触发随机目标发布（测试用） |

### Action

| Action | 类型 | 说明 |
|--------|------|------|
| `/execute_task_solution` | `moveit_task_constructor_msgs/ExecuteTaskSolution` | MTC 任务方案执行 |
| `/gantry_robot_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 轨迹跟随控制 |

### TF 帧

| 帧名 | 说明 |
|------|------|
| `world` | 世界坐标系 |
| `ipm` | 基准标记点 (Imaginary Point Marker) |
| `x_rail` | X 轴导轨原点 |
| `link1` ~ `link7` | 手臂连杆 |
| `link7_1` | 右臂末端执行器 |
| `cylinder_target_frame` | 目标圆柱体帧（由 target_scene_publisher 广播） |

## 坐标变换

MCU 原始值与 ROS SI 单位之间的变换在 `EngineerProtocol` 中处理：

| 关节 | MCU → ROS | ROS → MCU |
|------|-----------|------------|
| Joint 1 (Z 轴) | `-(raw / 1000) - 0.3` | `-(pos + 0.3) * 1000` |
| Joint 2, 3 (旋转) | `raw + 3.141592` | `pos - 3.141592` |
| Joint 4–6 | `raw`（直通） | `pos`（直通） |


## 操作流水线

行为树（`src/orchestrator/config/gantry_task.xml`）按顺序执行四个阶段：

### Phase 0 —— 对齐与加载

- 等待 `cylinder_target_frame` 与 `link7_1` 的 TF 变换
- 从当前状态移动到预加载位置（采样规划器，Z 轴 +3 cm 偏移）
- **模糊姿态生成**：在名义姿态周围采样 30 个含噪声的 IK 目标（位置/姿态容差内均匀噪声，偏航角 ±60° 范围）
- 下降到加载位置（笛卡尔规划器，Z 轴 -5 cm），将圆柱体附加到末端执行器上

### Phase 1 —— 推移

- 在规划场景中将圆柱体附加到末端执行器
- 沿圆柱体本地 Y 轴方向推移 0.1 m
- 使用减速采样规划器（0.1× 速度）确保安全运动
- **模糊姿态生成**：在推移目标姿态周围采样 50 个候选解

### Phase 2 —— 圆弧旋转（-90°）

- 分离圆柱体、重新定位、重新抓取
- 基于 **Rodrigues 旋转公式** 生成圆弧路径
- 旋转轴偏移：圆柱体原点 (y=108 mm, z=-169 mm)
- 12 个笛卡尔路径点，覆盖 -90° 旋转
- **降级策略**：规划失败时移动到最后可规划的路径点，然后回到 `home_gr`
- **关节运动代价排序**：大幅惩罚 X 导轨运动（65% 权重）vs. 臂关节运动（35% 权重）

### Phase 3 —— Y 轴旋转（+45°）

- 分离、重新定位、重新抓取
- 尝试两种可选规划策略：
  - **策略 A**：Pilz CIRC 圆弧运动规划器，带中间球体约束（半径 5 mm）
  - **策略 B**：24 步笛卡尔路径（步长 1.5 mm）
- 旋转轴：导出目标帧的本地 Y 轴，Z 偏移 -54 mm
- 规划失败时降级回到 `home_gr`

### 安全保护

- `recover_home_node` 以 5 Hz 频率监控末端执行器 Z 轴高度
- 若 Z 轴低于 `recover_z_threshold`（默认 45 mm），自动规划并执行回到 `home_gr` 的运动

## 串口通信协议

**物理层**: UART, 115200 baud, 8N1, `/dev/ttyACM0`

**数据帧** (`FixedPacket<64>`):

```
┌──────┬──────┬────────────┬──────────────┬───────┬──────┐
│ 0xFF │ mode │ positions  │  velocities  │ CRC16 │ 0x0D │
│ (1B) │ (1B) │  (6×4B)    │   (6×4B)     │ (2B)  │ (1B) │
└──────┴──────┴────────────┴──────────────┴───────┴──────┘
```

- **mode**: `3` = ros2_control 激活
- **positions/velocities**: float32，6 个关节
- **CRC-16**: CCITT 多项式
- MCU（mm，原始值）与 ROS（m，SI 单位）之间的坐标变换在 `EngineerProtocol` 中处理

**自动发送**: `EngineerProtocol` 订阅 `/display_planned_path` 话题，以 50 ms 间隔通过串口流式发送轨迹点。

## 快速开始

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 + MoveIt Task Constructor
- ros2_control + ros2_controllers
- BehaviorTree.CPP v4
- TRAC-IK 运动学插件
- Pilz industrial motion planner（可选，用于 Phase 3）

### 编译

```bash
cd ~/ws_sim
colcon build --symlink-install
source install/setup.bash
```

### 运行

**1. 真机模式（连接 MCU）**

```bash
# 启动 MoveIt + ros2_control
ros2 launch gantry_robot_moveit_config demo.launch.py

# 启动行为树编排器
ros2 launch orchestrator bt_orchestrator.launch.py

# （可选）发布目标场景物体
ros2 launch motion_planning_test target_scene_publisher.launch.py
```

**2. 仿真模式（无需硬件）**

```bash
# 以仿真模式启动 MoveIt
ros2 launch gantry_robot_moveit_config_sim demo.launch.py

# 启动行为树编排器
ros2 launch orchestrator bt_orchestrator.launch.py
```

**3. 双臂 "Engineer" 配置模式**

```bash
ros2 launch gantry_engineer_moveit_config demo.launch.py
ros2 launch orchestrator bt_orchestrator.launch.py
```

### 测试节点

```bash
# 末端对齐测试（龙门臂 → 台架）
ros2 run motion_planning_test ee_alignment_test

# 台架随机姿态测试
ros2 run motion_planning_test stand_random_pose

# 串口驱动测试
ros2 run orchestrator serial_test_node
```

## 可调参数

| 参数 | 位置 | 说明 |
|------|------|------|
| 串口设备 | `gantry_robot_hardware_interface` | `/dev/ttyACM0` —— 可切换至其他端口 |
| 关节限位 | `gantry_robot_moveit_config/config/joint_limits.yaml` | 速度/加速度限制 |
| 恢复 Z 阈值 | `recover_home_node` | 默认 0.045 m —— 触发自动恢复 |
| 模糊采样数 | Phase 0–3 节点 | 20–50 个样本 —— 越多规划越优但越慢 |
| MTC 规划尝试次数 | `MTC::doTask()` | 默认 5 —— 提高鲁棒性 |
| 速度缩放 | Phase 1 连接阶段 | 0.1 —— 推移过程中的安全减速 |
| 导轨运动代价权重 | `rankSolutionsByJointMotionCost()` | 0.65 —— 越大越倾向使用臂关节而非导轨 |
