#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>

class EndEffectorAlignmentTest : public rclcpp::Node
{
public:
  EndEffectorAlignmentTest() : Node("ee_alignment_test")
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "末端执行器世界坐标系对齐测试");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 创建TF2缓冲区和监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void run()
  {
    // 使用MoveGroupInterface控制gantry_robot
    auto gantry_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "gantry_robot");
    
    // 配置规划器参数
    gantry_move_group->setPlanningTime(30.0);  // 增加规划时间
    gantry_move_group->setNumPlanningAttempts(50);  // 增加尝试次数
    gantry_move_group->setMaxVelocityScalingFactor(0.6);  // 速度缩放因子
    gantry_move_group->setMaxAccelerationScalingFactor(0.6);  // 加速度缩放因子
    gantry_move_group->setGoalPositionTolerance(0.005);  // 5mm位置容差
    gantry_move_group->setGoalOrientationTolerance(0.05);  // ~2.86度姿态容差
    gantry_move_group->setGoalJointTolerance(0.01);  // 关节角度容差
    gantry_move_group->allowReplanning(true);  // 允许重新规划
    gantry_move_group->allowLooking(true);  // 允许环境感知
    
    // 获取规划场景接口用于碰撞检测
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    RCLCPP_INFO(this->get_logger(), "碰撞检测: 已启用");
    
    std::string gantry_ee_link = gantry_move_group->getEndEffectorLink();
    std::string reference_frame = gantry_move_group->getPlanningFrame();
    
    RCLCPP_INFO(this->get_logger(), "\n配置信息:");
    RCLCPP_INFO(this->get_logger(), "  Gantry规划组: %s", gantry_move_group->getName().c_str());
    RCLCPP_INFO(this->get_logger(), "  Gantry末端执行器: %s", gantry_ee_link.c_str());
    RCLCPP_INFO(this->get_logger(), "  参考坐标系: %s", reference_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  Stand末端执行器: cylinder3");
    
    // 等待TF变换稳定
    RCLCPP_INFO(this->get_logger(), "\n等待TF变换稳定...");
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    // 获取Gantry当前末端执行器在世界坐标系下的位姿
    geometry_msgs::msg::TransformStamped gantry_ee_transform_initial;
    try {
      gantry_ee_transform_initial = tf_buffer_->lookupTransform(
          reference_frame, gantry_ee_link, tf2::TimePointZero, std::chrono::seconds(5));
      
      RCLCPP_INFO(this->get_logger(), "\nGantry初始末端执行器位姿（世界坐标系）:");
      RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                  gantry_ee_transform_initial.transform.translation.x,
                  gantry_ee_transform_initial.transform.translation.y,
                  gantry_ee_transform_initial.transform.translation.z);
      printQuaternion("  姿态", gantry_ee_transform_initial.transform.rotation);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "无法获取Gantry初始位姿: %s", ex.what());
      return;
    }
    
    // 获取Stand末端执行器在世界坐标系下的位姿
    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::TransformStamped stand_ee_transform;
    tf2::Quaternion q_final_orientation;  // 最终对齐的姿态
    tf2::Vector3 final_z_axis;  // 最终的z轴方向
    
    try {
      std::string stand_ee_link = "cylinder3";
      
      if (!tf_buffer_->canTransform(reference_frame, stand_ee_link, tf2::TimePointZero, 
                                     std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "无法获取Stand末端执行器变换，超时！");
        return;
      }
      
      stand_ee_transform = tf_buffer_->lookupTransform(
          reference_frame, stand_ee_link, tf2::TimePointZero);
      
      RCLCPP_INFO(this->get_logger(), "\nStand目标末端执行器位姿（世界坐标系）:");
      RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                  stand_ee_transform.transform.translation.x,
                  stand_ee_transform.transform.translation.y,
                  stand_ee_transform.transform.translation.z);
      printQuaternion("  姿态", stand_ee_transform.transform.rotation);
      
      // 分析两个末端执行器的z轴方向
      RCLCPP_INFO(this->get_logger(), "\n========================================");
      RCLCPP_INFO(this->get_logger(), "Z轴方向分析");
      RCLCPP_INFO(this->get_logger(), "========================================");
      
      // 从四元数中提取z轴方向向量
      tf2::Quaternion q_gantry(
          gantry_ee_transform_initial.transform.rotation.x,
          gantry_ee_transform_initial.transform.rotation.y,
          gantry_ee_transform_initial.transform.rotation.z,
          gantry_ee_transform_initial.transform.rotation.w);
      
      tf2::Quaternion q_stand(
          stand_ee_transform.transform.rotation.x,
          stand_ee_transform.transform.rotation.y,
          stand_ee_transform.transform.rotation.z,
          stand_ee_transform.transform.rotation.w);
      
      // 提取z轴方向（局部坐标系的z轴在世界坐标系中的方向）
      tf2::Vector3 z_axis_local(0, 0, 1);
      tf2::Vector3 gantry_z_axis = tf2::quatRotate(q_gantry, z_axis_local);
      tf2::Vector3 stand_z_axis = tf2::quatRotate(q_stand, z_axis_local);
      
      RCLCPP_INFO(this->get_logger(), "Gantry末端执行器Z轴方向: [%.4f, %.4f, %.4f]",
                  gantry_z_axis.x(), gantry_z_axis.y(), gantry_z_axis.z());
      RCLCPP_INFO(this->get_logger(), "Stand末端执行器Z轴方向:  [%.4f, %.4f, %.4f]",
                  stand_z_axis.x(), stand_z_axis.y(), stand_z_axis.z());
      
      // 计算两个z轴的夹角（点积）
      double dot_product = gantry_z_axis.dot(stand_z_axis);
      double angle_rad = std::acos(std::max(-1.0, std::min(1.0, dot_product)));
      double angle_deg = angle_rad * 180.0 / M_PI;
      
      RCLCPP_INFO(this->get_logger(), "\nZ轴夹角: %.2f° (%.4f rad)", angle_deg, angle_rad);
      
      // 决定最终对齐的姿态（基于z轴夹角）
      tf2::Quaternion q_final_orientation;
      tf2::Vector3 final_z_axis;
      
      if (angle_deg < 90.0) {
        // 夹角小于90度，直接对齐
        RCLCPP_INFO(this->get_logger(), "\n策略: Z轴夹角 < 90°，直接对齐");
        q_final_orientation = q_stand;
        final_z_axis = stand_z_axis;
      } else {
        // 夹角大于等于90度，z轴反向对齐（绕x轴旋转180度）
        RCLCPP_INFO(this->get_logger(), "\n策略: Z轴夹角 >= 90°，Z轴反向对齐");
        
        // 创建一个绕x轴旋转180度的四元数
        tf2::Quaternion rotation_180;
        rotation_180.setRPY(M_PI, 0, 0);  // 绕X轴旋转180度
        
        // 将Stand的姿态与180度旋转组合
        q_final_orientation = q_stand * rotation_180;
        q_final_orientation.normalize();
        
        // 验证反向后的z轴方向
        final_z_axis = tf2::quatRotate(q_final_orientation, z_axis_local);
        RCLCPP_INFO(this->get_logger(), "反向后的Z轴方向: [%.4f, %.4f, %.4f]",
                    final_z_axis.x(), final_z_axis.y(), final_z_axis.z());
        
        double reversed_dot = gantry_z_axis.dot(final_z_axis);
        double reversed_angle = std::acos(std::max(-1.0, std::min(1.0, reversed_dot))) * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "反向后与Gantry的Z轴夹角: %.2f°", reversed_angle);
      }
      
      // ========================================
      // 第一阶段：运动到目标上方0.1米，坐标轴对齐
      // ========================================
      RCLCPP_INFO(this->get_logger(), "\n========================================");
      RCLCPP_INFO(this->get_logger(), "第一阶段：接近位置");
      RCLCPP_INFO(this->get_logger(), "========================================");
      
      // 构建第一阶段目标位姿：沿stand的z轴正方向偏移0.1米
      target_pose.header.frame_id = reference_frame;
      target_pose.header.stamp = this->now();
      
      // 位置：沿stand的z轴方向偏移0.1米
      target_pose.pose.position.x = stand_ee_transform.transform.translation.x + final_z_axis.x() * 0.1;
      target_pose.pose.position.y = stand_ee_transform.transform.translation.y + final_z_axis.y() * 0.1;
      target_pose.pose.position.z = stand_ee_transform.transform.translation.z + final_z_axis.z() * 0.1;
      
      // 姿态：与最终目标姿态一致
      target_pose.pose.orientation.x = q_final_orientation.x();
      target_pose.pose.orientation.y = q_final_orientation.y();
      target_pose.pose.orientation.z = q_final_orientation.z();
      target_pose.pose.orientation.w = q_final_orientation.w();
      
      RCLCPP_INFO(this->get_logger(), "\n第一阶段目标位姿（z轴+0.1m）:");
      RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                  target_pose.pose.position.x,
                  target_pose.pose.position.y,
                  target_pose.pose.position.z);
      printQuaternion("  姿态", target_pose.pose.orientation);
      
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "TF查找失败: %s", ex.what());
      return;
    }
    
    // 先测试IK是否有解，使用多种策略
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "测试运动学求解（IK）...");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    moveit::core::RobotStatePtr current_state = gantry_move_group->getCurrentState(10.0);
    const moveit::core::JointModelGroup* joint_model_group = 
        current_state->getJointModelGroup(gantry_move_group->getName());
    
    bool ik_found = false;
    geometry_msgs::msg::Pose final_target_pose = target_pose.pose;
    
    // 策略1: 尝试原始目标姿态
    RCLCPP_INFO(this->get_logger(), "尝试策略1: 使用原始目标姿态...");
    ik_found = current_state->setFromIK(joint_model_group, target_pose.pose, 10.0);
    
    if (!ik_found) {
      RCLCPP_WARN(this->get_logger(), "策略1失败，尝试策略2: 仅位置约束（放松姿态）...");
      
      // 策略2: 尝试仅调整姿态但保持目标位置
      // 尝试从当前姿态逐步调整到目标姿态
      tf2::Quaternion q_current(
          gantry_ee_transform_initial.transform.rotation.x,
          gantry_ee_transform_initial.transform.rotation.y,
          gantry_ee_transform_initial.transform.rotation.z,
          gantry_ee_transform_initial.transform.rotation.w);
      
      tf2::Quaternion q_target(
          target_pose.pose.orientation.x,
          target_pose.pose.orientation.y,
          target_pose.pose.orientation.z,
          target_pose.pose.orientation.w);
      
      // 尝试插值姿态
      for (double t = 0.2; t <= 1.0 && !ik_found; t += 0.2) {
        tf2::Quaternion q_interpolated = q_current.slerp(q_target, t);
        q_interpolated.normalize();
        
        geometry_msgs::msg::Pose interpolated_pose = target_pose.pose;
        interpolated_pose.orientation.x = q_interpolated.x();
        interpolated_pose.orientation.y = q_interpolated.y();
        interpolated_pose.orientation.z = q_interpolated.z();
        interpolated_pose.orientation.w = q_interpolated.w();
        
        ik_found = current_state->setFromIK(joint_model_group, interpolated_pose, 5.0);
        
        if (ik_found) {
          RCLCPP_INFO(this->get_logger(), "✓ 策略2成功：使用插值姿态 (t=%.1f)", t);
          final_target_pose = interpolated_pose;
          break;
        }
      }
    }
    
    if (!ik_found) {
      RCLCPP_WARN(this->get_logger(), "策略2失败，尝试策略3: 接近目标位置（偏移）...");
      
      // 策略3: 在目标上方/周围偏移
      std::vector<std::array<double, 3>> offsets = {
          {0.0, 0.0, 0.05},   // 上方5cm
          {0.0, 0.0, 0.10},   // 上方10cm
          {0.05, 0.0, 0.05},  // 右上
          {-0.05, 0.0, 0.05}, // 左上
          {0.0, 0.05, 0.05},  // 前上
          {0.0, -0.05, 0.05}  // 后上
      };
      
      for (const auto& offset : offsets) {
        geometry_msgs::msg::Pose offset_pose = target_pose.pose;
        offset_pose.position.x += offset[0];
        offset_pose.position.y += offset[1];
        offset_pose.position.z += offset[2];
        
        ik_found = current_state->setFromIK(joint_model_group, offset_pose, 5.0);
        
        if (ik_found) {
          RCLCPP_INFO(this->get_logger(), "✓ 策略3成功：使用偏移位置 [%.3f, %.3f, %.3f]",
                      offset[0], offset[1], offset[2]);
          final_target_pose = offset_pose;
          break;
        }
      }
    }
    
    if (ik_found) {
      RCLCPP_INFO(this->get_logger(), "\n✓✓ IK求解成功！目标位姿可达");
      
      // 显示关节角度
      std::vector<double> joint_values;
      current_state->copyJointGroupPositions(joint_model_group, joint_values);
      RCLCPP_INFO(this->get_logger(), "关节角度解:");
      for (size_t i = 0; i < joint_values.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.4f rad (%.2f°)", 
                    i+1, joint_values[i], joint_values[i] * 180.0 / M_PI);
      }
      
      RCLCPP_INFO(this->get_logger(), "\n最终目标位姿:");
      RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                  final_target_pose.position.x,
                  final_target_pose.position.y,
                  final_target_pose.position.z);
      
      // 重新设置目标
      gantry_move_group->clearPoseTargets();
      gantry_move_group->setPoseTarget(final_target_pose);
      
    } else {
      RCLCPP_ERROR(this->get_logger(), "\n✗✗✗ 所有IK策略均失败！");
      RCLCPP_ERROR(this->get_logger(), "原始目标位置: [%.4f, %.4f, %.4f]",
                   target_pose.pose.position.x,
                   target_pose.pose.position.y,
                   target_pose.pose.position.z);
      RCLCPP_ERROR(this->get_logger(), "\n可能的原因:");
      RCLCPP_ERROR(this->get_logger(), "  1. 目标位置超出机器人工作空间");
      RCLCPP_ERROR(this->get_logger(), "  2. 目标姿态导致关节超限");
      RCLCPP_ERROR(this->get_logger(), "  3. 存在自碰撞风险");
      RCLCPP_ERROR(this->get_logger(), "\n建议:");
      RCLCPP_ERROR(this->get_logger(), "  - 在RViz中手动拖动检查目标是否可达");
      RCLCPP_ERROR(this->get_logger(), "  - 检查关节限位配置");
      RCLCPP_ERROR(this->get_logger(), "  - 调整Stand和Gantry的初始位置");
      return;
    }
    
    // ========================================
    // 第一阶段规划和执行
    // ========================================
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "第一阶段：规划到接近位置");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan_phase1;
    bool success_phase1 = (gantry_move_group->plan(plan_phase1) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!success_phase1) {
      RCLCPP_ERROR(this->get_logger(), "✗ 第一阶段规划失败！");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✓ 第一阶段规划成功！");
    RCLCPP_INFO(this->get_logger(), "  轨迹点数: %zu", plan_phase1.trajectory.joint_trajectory.points.size());
    RCLCPP_INFO(this->get_logger(), "  规划时间: %.2f 秒", plan_phase1.planning_time);
    
    // 执行第一阶段运动
    RCLCPP_INFO(this->get_logger(), "\n执行第一阶段运动...");
    auto execute_result_phase1 = gantry_move_group->execute(plan_phase1);
    
    if (execute_result_phase1 != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "✗ 第一阶段运动执行失败！");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✓ 第一阶段运动执行成功！");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ========================================
    // 第二阶段：沿z轴负方向直线运动0.1米
    // ========================================
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "第二阶段：笛卡尔直线运动");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 构建第二阶段目标位姿：沿z轴负方向移动0.1米
    geometry_msgs::msg::Pose phase2_target_pose;
    phase2_target_pose.position.x = stand_ee_transform.transform.translation.x;
    phase2_target_pose.position.y = stand_ee_transform.transform.translation.y;
    phase2_target_pose.position.z = stand_ee_transform.transform.translation.z;
    
    phase2_target_pose.orientation.x = q_final_orientation.x();
    phase2_target_pose.orientation.y = q_final_orientation.y();
    phase2_target_pose.orientation.z = q_final_orientation.z();
    phase2_target_pose.orientation.w = q_final_orientation.w();
    
    RCLCPP_INFO(this->get_logger(), "第二阶段目标位姿（最终位置）:");
    RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                phase2_target_pose.position.x,
                phase2_target_pose.position.y,
                phase2_target_pose.position.z);
    
    // 使用笛卡尔路径规划
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(phase2_target_pose);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1cm步长
    
    
    double fraction = gantry_move_group->computeCartesianPath(
        waypoints, eef_step, trajectory);
    
    if (fraction < 0.95) {
      RCLCPP_WARN(this->get_logger(), "⚠ 笛卡尔路径规划部分成功: %.1f%%", fraction * 100.0);
      RCLCPP_WARN(this->get_logger(), "尝试使用标准规划器...");
      
      // 回退到标准规划
      gantry_move_group->setPoseTarget(phase2_target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan_phase2;
      bool success_phase2 = (gantry_move_group->plan(plan_phase2) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (!success_phase2) {
        RCLCPP_ERROR(this->get_logger(), "✗ 第二阶段规划失败！");
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "✓ 使用标准规划器成功");
      
      // 执行第二阶段
      auto execute_result_phase2 = gantry_move_group->execute(plan_phase2);
      
      if (execute_result_phase2 != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "✗ 第二阶段运动执行失败！");
        return;
      }
      
    } else {
      RCLCPP_INFO(this->get_logger(), "✓ 笛卡尔路径规划成功: %.1f%%", fraction * 100.0);
      
      // 执行笛卡尔路径
      moveit::planning_interface::MoveGroupInterface::Plan plan_phase2;
      plan_phase2.trajectory = trajectory;
      
      auto execute_result_phase2 = gantry_move_group->execute(plan_phase2);
      
      if (execute_result_phase2 != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "✗ 第二阶段运动执行失败！");
        return;
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "✓ 第二阶段运动执行成功！");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // 验证最终对齐精度
    RCLCPP_INFO(this->get_logger(), "\n========================================");
    RCLCPP_INFO(this->get_logger(), "验证最终对齐精度");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    verifyAlignment(gantry_ee_link, stand_ee_transform, reference_frame);
  }

private:
  void printQuaternion(const std::string& label, const geometry_msgs::msg::Quaternion& q)
  {
    // 转换为RPY角度便于理解
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(), "%s: [x:%.4f, y:%.4f, z:%.4f, w:%.4f]",
                label.c_str(), q.x, q.y, q.z, q.w);
    RCLCPP_INFO(this->get_logger(), "       RPY: [%.2f°, %.2f°, %.2f°]",
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
  }
  
  void verifyAlignment(const std::string& gantry_ee_link, 
                      const geometry_msgs::msg::TransformStamped& stand_transform,
                      const std::string& reference_frame)
  {
    try {
      // 获取Gantry最终末端执行器位姿
      auto gantry_final_transform = tf_buffer_->lookupTransform(
          reference_frame, gantry_ee_link, tf2::TimePointZero, std::chrono::seconds(2));
      
      RCLCPP_INFO(this->get_logger(), "\nGantry最终末端执行器位姿（世界坐标系）:");
      RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                  gantry_final_transform.transform.translation.x,
                  gantry_final_transform.transform.translation.y,
                  gantry_final_transform.transform.translation.z);
      printQuaternion("  姿态", gantry_final_transform.transform.rotation);
      
      RCLCPP_INFO(this->get_logger(), "\nStand末端执行器位姿（世界坐标系）:");
      RCLCPP_INFO(this->get_logger(), "  位置: [%.4f, %.4f, %.4f]",
                  stand_transform.transform.translation.x,
                  stand_transform.transform.translation.y,
                  stand_transform.transform.translation.z);
      printQuaternion("  姿态", stand_transform.transform.rotation);
      
      // 计算位置误差（欧几里得距离）
      double dx = gantry_final_transform.transform.translation.x - stand_transform.transform.translation.x;
      double dy = gantry_final_transform.transform.translation.y - stand_transform.transform.translation.y;
      double dz = gantry_final_transform.transform.translation.z - stand_transform.transform.translation.z;
      double position_error = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      // 计算姿态误差（四元数角度差）
      tf2::Quaternion q_gantry(
          gantry_final_transform.transform.rotation.x,
          gantry_final_transform.transform.rotation.y,
          gantry_final_transform.transform.rotation.z,
          gantry_final_transform.transform.rotation.w);
      
      tf2::Quaternion q_stand(
          stand_transform.transform.rotation.x,
          stand_transform.transform.rotation.y,
          stand_transform.transform.rotation.z,
          stand_transform.transform.rotation.w);
      
      // 计算相对旋转
      tf2::Quaternion q_diff = q_stand.inverse() * q_gantry;
      double orientation_error = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.w())));
      
      RCLCPP_INFO(this->get_logger(), "\n对齐误差分析:");
      RCLCPP_INFO(this->get_logger(), "  位置误差:");
      RCLCPP_INFO(this->get_logger(), "    ΔX: %.4f m (%.1f mm)", dx, dx * 1000.0);
      RCLCPP_INFO(this->get_logger(), "    ΔY: %.4f m (%.1f mm)", dy, dy * 1000.0);
      RCLCPP_INFO(this->get_logger(), "    ΔZ: %.4f m (%.1f mm)", dz, dz * 1000.0);
      RCLCPP_INFO(this->get_logger(), "    总误差: %.4f m (%.1f mm)", position_error, position_error * 1000.0);
      
      RCLCPP_INFO(this->get_logger(), "  姿态误差:");
      RCLCPP_INFO(this->get_logger(), "    角度差: %.4f rad (%.2f°)", 
                  orientation_error, orientation_error * 180.0 / M_PI);
      
      // 判定对齐质量
      RCLCPP_INFO(this->get_logger(), "\n对齐结果:");
      if (position_error < 0.001 && orientation_error < 0.01) {
        RCLCPP_INFO(this->get_logger(), "  ✓✓✓ 精确对齐！位置和姿态误差都在优秀范围内");
      } else if (position_error < 0.005 && orientation_error < 0.05) {
        RCLCPP_INFO(this->get_logger(), "  ✓✓ 良好对齐！位置和姿态误差在可接受范围内");
      } else if (position_error < 0.01 && orientation_error < 0.1) {
        RCLCPP_WARN(this->get_logger(), "  ✓ 基本对齐，但存在一定误差");
      } else {
        RCLCPP_ERROR(this->get_logger(), "  ✗ 对齐质量较差，请检查：");
        RCLCPP_ERROR(this->get_logger(), "    - 运动学精度");
        RCLCPP_ERROR(this->get_logger(), "    - 规划器收敛条件");
        RCLCPP_ERROR(this->get_logger(), "    - 关节限位是否影响了位姿");
      }
      
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "验证失败 - TF查找错误: %s", ex.what());
    }
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  // 初始化ROS 2
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto node = std::make_shared<EndEffectorAlignmentTest>();
  
  // 创建单线程执行器
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // 启动执行器线程
  std::thread executor_thread([&executor]() { executor.spin(); });
  
  // 运行测试
  node->run();
  
  // 关闭执行器
  executor.cancel();
  executor_thread.join();
  
  rclcpp::shutdown();
  return 0;
}
