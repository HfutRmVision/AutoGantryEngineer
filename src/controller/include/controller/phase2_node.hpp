#ifndef PHASE2_NODE_HPP
#define PHASE2_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/task_constructor/stages/predicate_filter.h"
#include <limits>
#include <string>
#include <vector>

/**
 * @brief Phase 2 - 圆弧旋转阶段
 * 
 * 功能：目标圆柱体在“圆柱 z 轴 + 相对定义轴”构成的平面内绕圆心旋转 -90°
 * - 旋转轴原点：取相对定义轴上离目标物体坐标（圆柱体原点）最近的一点
 * - 旋转轴：相对定义轴（与局部 x 轴平行）
 * - 旋转角度：-90 度（顺时针）
 * 
 * 实现方案：
 * 1) 按 P 轴参数生成圆弧位置 waypoint（仅位置遵循圆弧）；
 * 2) 每个 waypoint 的运动规划交给 fuzzy pose + OMPL（Connect + ComputeIK）；
 * 3) 姿态不再由 CIRC 刚性插值锁死，yaw 放开由 FuzzyPoseGenerator 完成。
 */
class Phase2Node : public rclcpp::Node, public MTC {
public:
    Phase2Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc"),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        RCLCPP_INFO(this->get_logger(), "Phase2: Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero, std::chrono::seconds(1));
                tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Phase2: TF available");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", e.what());
            }
        }
        
        if (!doTask()) {
            RCLCPP_WARN(this->get_logger(), "Phase2: Task failed, executing fallback (last-plannable + home)");
            doFallback();
        }
    }

private:
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<geometry_msgs::msg::PoseStamped> arc_waypoints_cache_;

    geometry_msgs::msg::PoseStamped getCurrentLink7Pose() {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = this->now();

        try {
            const auto ee_tf = tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero);
            pose.pose.position.x = ee_tf.transform.translation.x;
            pose.pose.position.y = ee_tf.transform.translation.y;
            pose.pose.position.z = ee_tf.transform.translation.z;
            pose.pose.orientation = ee_tf.transform.rotation;
        } catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(), "Phase2: Could not get current link7 pose: %s", e.what());
        }

        return pose;
    }
    
    // 相对定义轴参数（相对于圆柱体局部坐标）
    static constexpr double P_AXIS_Y_OFFSET = 0.108;   // 相对轴参考点 y 偏移
    static constexpr double P_AXIS_Z_OFFSET = -0.169;  // 相对轴参考点 z 偏移
    static constexpr double ROTATION_ANGLE = -M_PI_2;  // -90度
    static constexpr double CYLINDER_BODY_Z_OFFSET_FROM_LINK7 = 0.05;  // attached cylinder body frame relative to link7
    static constexpr int ARC_WAYPOINT_COUNT = 12;       // 圆弧离散点数量（含起点终点）
    static constexpr bool ARC_IGNORE_YAW = false;        // waypoint 生成时忽略 yaw（统一到起点 yaw）

    /**
     * @brief 绕任意轴旋转位姿（位置与姿态均按刚体旋转）
     */
    geometry_msgs::msg::PoseStamped rotatePoseAroundAxis(
        const geometry_msgs::msg::PoseStamped& pose_world,
        const tf2::Vector3& axis_origin_world,
        const tf2::Vector3& axis_dir_world,
        double angle) const
    {
        tf2::Vector3 point_world(
            pose_world.pose.position.x,
            pose_world.pose.position.y,
            pose_world.pose.position.z);
        tf2::Quaternion ori_world(
            pose_world.pose.orientation.x,
            pose_world.pose.orientation.y,
            pose_world.pose.orientation.z,
            pose_world.pose.orientation.w);

        // 位置：罗德里格斯公式
        tf2::Vector3 r = point_world - axis_origin_world;
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        tf2::Vector3 r_rot = r * cos_a
                            + axis_dir_world.cross(r) * sin_a
                            + axis_dir_world * axis_dir_world.dot(r) * (1.0 - cos_a);
        tf2::Vector3 new_pos = axis_origin_world + r_rot;

        tf2::Quaternion rot_q;
        rot_q.setRotation(axis_dir_world, angle);
        tf2::Quaternion new_ori = rot_q * ori_world;
        new_ori.normalize();

        geometry_msgs::msg::PoseStamped result = pose_world;
        result.header.frame_id = "world";
        result.pose.position.x = new_pos.x();
        result.pose.position.y = new_pos.y();
        result.pose.position.z = new_pos.z();
        result.pose.orientation = tf2::toMsg(new_ori);
        return result;
    }

    /**
     * @brief 生成圆弧路径 waypoint（位置与姿态同步绕轴旋转）
     */
    std::vector<geometry_msgs::msg::PoseStamped> generateArcWaypoints(
        const geometry_msgs::msg::PoseStamped& start_pose,
        const tf2::Vector3& axis_origin_world,
        const tf2::Vector3& axis_dir_world,
        double rotation_angle,
        int waypoint_count,
        bool ignore_yaw) const
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        if (waypoint_count < 2) {
            waypoints.push_back(start_pose);
            return waypoints;
        }

        double start_roll = 0.0;
        double start_pitch = 0.0;
        double start_yaw = 0.0;
        if (ignore_yaw) {
            tf2::Quaternion q_start(
                start_pose.pose.orientation.x,
                start_pose.pose.orientation.y,
                start_pose.pose.orientation.z,
                start_pose.pose.orientation.w);
            tf2::Matrix3x3(q_start).getRPY(start_roll, start_pitch, start_yaw);
        }

        waypoints.reserve(static_cast<size_t>(waypoint_count));

        for (int i = 0; i < waypoint_count; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(waypoint_count - 1);
            const double angle = rotation_angle * ratio;
            auto waypoint = rotatePoseAroundAxis(start_pose, axis_origin_world, axis_dir_world, angle);

            if (ignore_yaw) {
                tf2::Quaternion q_wp(
                    waypoint.pose.orientation.x,
                    waypoint.pose.orientation.y,
                    waypoint.pose.orientation.z,
                    waypoint.pose.orientation.w);
                double roll = 0.0;
                double pitch = 0.0;
                double yaw = 0.0;
                tf2::Matrix3x3(q_wp).getRPY(roll, pitch, yaw);

                tf2::Quaternion q_yaw_free;
                q_yaw_free.setRPY(roll, pitch, start_yaw);
                q_yaw_free.normalize();
                waypoint.pose.orientation = tf2::toMsg(q_yaw_free);
            }

            waypoints.push_back(waypoint);
        }

        return waypoints;
    }

    geometry_msgs::msg::PoseStamped deriveAttachedCylinderBodyPose(
        const geometry_msgs::msg::PoseStamped& ee_world) const {
        geometry_msgs::msg::PoseStamped cylinder_body_pose;
        cylinder_body_pose.header.frame_id = "world";
        cylinder_body_pose.header.stamp = this->now();

        try {
            // 1. 获取目标最原始的 TF 位姿
            geometry_msgs::msg::TransformStamped cyl_tf = 
                tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero);

            // 提取原本的旋转和平移
            tf2::Quaternion q(
                cyl_tf.transform.rotation.x,
                cyl_tf.transform.rotation.y,
                cyl_tf.transform.rotation.z,
                cyl_tf.transform.rotation.w
            );
            tf2::Vector3 origin(
                cyl_tf.transform.translation.x,
                cyl_tf.transform.translation.y,
                cyl_tf.transform.translation.z
            );

            // 2. 按你的思路：在圆柱体局部坐标系下，向 Y 轴偏移 +0.1m
            tf2::Vector3 y_offset_local(0.0, 0.1, 0.0);
            
            // 将局部偏移量转换到世界坐标系下
            tf2::Vector3 y_offset_world = tf2::quatRotate(q, y_offset_local);

            // 3. 计算推演后的真实原点位置 = 原始位置 + 偏移量
            tf2::Vector3 new_origin = origin + y_offset_world;

            // 4. 组装最终的真实目标位姿 (姿态保持原始 TF 姿态，位置更新为推移后的位置)
            cylinder_body_pose.pose.position.x = new_origin.x();
            cylinder_body_pose.pose.position.y = new_origin.y();
            cylinder_body_pose.pose.position.z = new_origin.z();
            cylinder_body_pose.pose.orientation = cyl_tf.transform.rotation;

            RCLCPP_INFO(this->get_logger(), "Phase2: Cylinder pose estimated via TF +0.1m Y-offset. Pos: (%.3f, %.3f, %.3f)",
                cylinder_body_pose.pose.position.x,
                cylinder_body_pose.pose.position.y,
                cylinder_body_pose.pose.position.z);

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN(this->get_logger(), "Phase2: TF lookup failed: %s. Falling back to link7 approximation.", e.what());
            // 万一系统刚起没抓到 TF，用 link7 兜个底防止节点直接崩溃
            cylinder_body_pose = ee_world;
            tf2::Quaternion ee_q(ee_world.pose.orientation.x, ee_world.pose.orientation.y, ee_world.pose.orientation.z, ee_world.pose.orientation.w);
            tf2::Vector3 body_offset_local(0.0, 0.0, CYLINDER_BODY_Z_OFFSET_FROM_LINK7);
            tf2::Vector3 body_offset_world = tf2::quatRotate(ee_q, body_offset_local);
            cylinder_body_pose.pose.position.x += body_offset_world.x();
            cylinder_body_pose.pose.position.y += body_offset_world.y();
            cylinder_body_pose.pose.position.z += body_offset_world.z();
        }

        return cylinder_body_pose;
    }

    struct PlaneRotationGeometry {
        tf2::Vector3 center_world;
        tf2::Vector3 rot_axis_world;
        tf2::Vector3 cylinder_z_world;
        tf2::Vector3 relative_axis_world;
        double line_gap_m{0.0};
    };

    PlaneRotationGeometry computePhase2Geometry(
        const geometry_msgs::msg::PoseStamped& attached_cylinder_body_pose) const
    {
        tf2::Quaternion cylinder_body_q(
            attached_cylinder_body_pose.pose.orientation.x,
            attached_cylinder_body_pose.pose.orientation.y,
            attached_cylinder_body_pose.pose.orientation.z,
            attached_cylinder_body_pose.pose.orientation.w);
        tf2::Vector3 cylinder_body_origin(
            attached_cylinder_body_pose.pose.position.x,
            attached_cylinder_body_pose.pose.position.y,
            attached_cylinder_body_pose.pose.position.z);

        // 1. 旋转轴方向：在 target 坐标系下平行于 x 轴 (1, 0, 0)
        tf2::Vector3 p_axis_dir_local(1.0, 0.0, 0.0);
        tf2::Vector3 p_axis_dir_world = tf2::quatRotate(cylinder_body_q, p_axis_dir_local);
        p_axis_dir_world.normalize();

        // 2. 旋转轴原点：在 target 坐标系下偏移 y=+108mm, z=-169mm
        // 因为方向平行于 x，所以 x=0 的这个点就是该轴上离 target 原点最近的点
        tf2::Vector3 p_axis_origin_local(0.0, P_AXIS_Y_OFFSET, P_AXIS_Z_OFFSET);
        tf2::Vector3 p_axis_origin_world = cylinder_body_origin + tf2::quatRotate(cylinder_body_q, p_axis_origin_local);

        // 3. 计算轴到原点的实际距离 (用于 debug 确认)
        double line_gap = p_axis_origin_local.length(); // 局部坐标系下的模长即为距离

        PlaneRotationGeometry geometry;
        geometry.center_world = p_axis_origin_world;  // 使用该点作为旋转中心
        geometry.rot_axis_world = p_axis_dir_world;   // 旋转轴方向
        geometry.relative_axis_world = p_axis_dir_world;
        geometry.line_gap_m = line_gap;
        
        // 如果还需要 cylinder_z_world 可以保留
        geometry.cylinder_z_world = tf2::quatRotate(cylinder_body_q, tf2::Vector3(0.0, 0.0, 1.0)).normalized();

        return geometry;
    }

    void doFallback() {
        // 从规划场景中释放物体
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::AttachedCollisionObject detach_obj;
        detach_obj.object.id = "cylinder_target";
        detach_obj.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        detach_obj.link_name = "link7_1";
        psi.applyAttachedCollisionObject(detach_obj);
        RCLCPP_INFO(this->get_logger(), "Phase2 fallback: object detached");

        // 返回 home_gr
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            MTC::getNodeSharedPtr(), "right_arm");
        move_group->setMaxVelocityScalingFactor(0.1);
        move_group->setMaxAccelerationScalingFactor(0.1);
        move_group->setPlanningTime(15.0);
        move_group->setNumPlanningAttempts(5);

        bool moved_to_last_planned = false;
        for (size_t i = arc_waypoints_cache_.size(); i > 1 && rclcpp::ok(); --i) {
            const auto& target_pose = arc_waypoints_cache_[i - 1];
            move_group->setStartStateToCurrentState();
            move_group->setPoseTarget(target_pose.pose, "link7_1");

            moveit::planning_interface::MoveGroupInterface::Plan waypoint_plan;
            const auto plan_result = move_group->plan(waypoint_plan);
            if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                continue;
            }

            RCLCPP_INFO(this->get_logger(),
                "Phase2 fallback: moving to last plannable arc waypoint #%zu", i - 1);
            const auto exec_result = move_group->execute(waypoint_plan);
            move_group->clearPoseTargets();
            if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                moved_to_last_planned = true;
                break;
            }
        }

        if (moved_to_last_planned) {
            RCLCPP_INFO(this->get_logger(),
                "Phase2 fallback: reached last plannable arc waypoint successfully");
            return;
        }

        RCLCPP_WARN(this->get_logger(),
            "Phase2 fallback: failed to reach any arc waypoint, fallback to home_gr");

        constexpr int max_retry = 5;
        bool moved_home = false;
        for (int attempt = 1; attempt <= max_retry && rclcpp::ok(); ++attempt) {
            move_group->setStartStateToCurrentState();
            move_group->setNamedTarget("home_gr");

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto plan_result = move_group->plan(plan);
            if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(this->get_logger(),
                    "Phase2 fallback: planning to home_gr failed (attempt %d/%d)",
                    attempt, max_retry);
                continue;
            }

            RCLCPP_INFO(this->get_logger(),
                "Phase2 fallback: moving to home_gr (attempt %d/%d)",
                attempt, max_retry);
            auto exec_result = move_group->execute(plan);
            if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
                moved_home = true;
                break;
            }

            RCLCPP_WARN(this->get_logger(),
                "Phase2 fallback: execution to home_gr failed (attempt %d/%d)",
                attempt, max_retry);
        }

        if (!moved_home) {
            RCLCPP_ERROR(this->get_logger(),
                "Phase2 fallback: failed to move to home_gr after %d attempts", max_retry);
        }
    }

    void setup_planning_scene() override {}

    mtc::Task create_task() override {
        mtc::Task task = TaskInit("circular_rotation");

        // ============================================================
        // 通用采样规划器（用于回 home）
        // ============================================================
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxVelocityScalingFactor(0.1);
        sampling_planner->setMaxAccelerationScalingFactor(0.1);

        geometry_msgs::msg::PoseStamped reload_goal_pose;

        // Stage 1: 当前状态
        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            task.add(std::move(stage));
        }

        mtc::Stage* detach_state_ptr = nullptr;
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach before reload");
            stage->detachObject("cylinder_target", "link7_1");
            detach_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        // Stage 2: move to reload seed (sampling connect)
        {
            mtc::stages::Connect::GroupPlannerVector planners = {
                {"right_arm", sampling_planner}
            };
            auto stage = std::make_unique<mtc::stages::Connect>("move to reload", planners);
            stage->setTimeout(10.0);

            auto filter = std::make_unique<mtc::stages::PredicateFilter>("reload pose filter", std::move(stage));
            filter->setPredicate([](const mtc::SolutionBase& s, std::string& comment) {
                if(s.cost() < 0.05) {
                    comment = "Solution cost is too low.";
                    return false;
                }
                return true;
            });

            task.add(std::move(filter));
        }

        // Stage 3: reload（参考 Phase0 的 load）
        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator for reloading");
            generator->setSampleCount(30);
            generator->setTolerance(0.01, 0.2);

            // Reload against the actual object pose after Phase1 push.
            // The continuously published target frame can stay at the pre-push pose.
            reload_goal_pose = getCurrentLink7Pose();

            generator->setPose(reload_goal_pose);
            generator->setMonitoredStage(detach_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto stage = std::make_unique<mtc::stages::ComputeIK>("compute ik for reloading", std::move(generator));
            stage->setMaxIKSolutions(2);
            stage->setMinSolutionDistance(2);
            stage->setIKFrame("link7_1");
            stage->setTargetPose(reload_goal_pose);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setIgnoreCollisions(false);
            task.add(std::move(stage));
        }

        mtc::Stage* attach_state_ptr = nullptr;
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->allowCollisions("cylinder_target", "right_arm", true);
            stage->attachObject("cylinder_target", "link7_1");
            attach_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        
        // ============================================================
        // 计算圆弧 waypoint：起点到终点（仅位置沿圆弧）
        // ============================================================
        geometry_msgs::msg::PoseStamped current_pose = reload_goal_pose;
        geometry_msgs::msg::PoseStamped attached_cylinder_body_pose = deriveAttachedCylinderBodyPose(current_pose);
        const auto geometry = computePhase2Geometry(attached_cylinder_body_pose);

        tf2::Vector3 ee_pos(current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z);
        tf2::Vector3 r = ee_pos - geometry.center_world;
        tf2::Vector3 r_perp = r - geometry.rot_axis_world * r.dot(geometry.rot_axis_world);
        double radius = r_perp.length();

        auto arc_waypoints = generateArcWaypoints(
            current_pose,
            geometry.center_world,
            geometry.rot_axis_world,
            ROTATION_ANGLE,
            ARC_WAYPOINT_COUNT,
            ARC_IGNORE_YAW);
        arc_waypoints_cache_ = arc_waypoints;

        RCLCPP_INFO(this->get_logger(),
            "Phase2: Arc waypoint generation (radius: %.3f m, angle: %.1f deg, waypoints: %zu, axis-gap: %.6f m, ignore-yaw: %s)",
            radius,
            ROTATION_ANGLE * 180.0 / M_PI,
            arc_waypoints.size(),
            geometry.line_gap_m,
            ARC_IGNORE_YAW ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Phase2: Reload pose from current link7: (%.4f, %.4f, %.4f)",
            reload_goal_pose.pose.position.x,
            reload_goal_pose.pose.position.y,
            reload_goal_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Attached cylinder body: (%.4f, %.4f, %.4f)",
            attached_cylinder_body_pose.pose.position.x,
            attached_cylinder_body_pose.pose.position.y,
            attached_cylinder_body_pose.pose.position.z);
        if (!arc_waypoints.empty()) {
            const auto& first_wp = arc_waypoints.front();
            const auto& last_wp = arc_waypoints.back();
            RCLCPP_INFO(this->get_logger(), "  Circle center: (%.4f, %.4f, %.4f)",
                geometry.center_world.x(), geometry.center_world.y(), geometry.center_world.z());
            RCLCPP_INFO(this->get_logger(), "  Rotation axis: (%.4f, %.4f, %.4f)",
                geometry.rot_axis_world.x(), geometry.rot_axis_world.y(), geometry.rot_axis_world.z());
            RCLCPP_INFO(this->get_logger(), "  Start:   (%.4f, %.4f, %.4f)",
                first_wp.pose.position.x, first_wp.pose.position.y, first_wp.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "  Goal:    (%.4f, %.4f, %.4f)",
                last_wp.pose.position.x, last_wp.pose.position.y, last_wp.pose.position.z);
        }

        // ============================================================
        // Stage 2..N: waypoint-by-waypoint（fuzzy pose + OMPL）
        // ============================================================
        mtc::Stage* arc_seed_ptr = attach_state_ptr ? attach_state_ptr : detach_state_ptr;
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.5);
        cartesian_planner->setMaxAccelerationScalingFactor(0.5);
        cartesian_planner->setStepSize(0.001); // 步长 1cm 即可
        cartesian_planner->setIKFrame("link7_1");

        // 顺次添加 MoveTo 阶段，状态数量变成 1+1+1... 而不是 1*160*160...
        for (size_t i = 1; i < arc_waypoints.size(); ++i) {
            auto stage = std::make_unique<mtc::stages::MoveTo>(
                "phase2_arc_wp_" + std::to_string(i), cartesian_planner);
            
            stage->setIKFrame("link7_1");
            stage->setTimeout(3.0);
            stage->setGoal(arc_waypoints[i]); // 直接使用你算好的精准位姿
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            
            task.add(std::move(stage));
        }

        // Stage 3: 释放物体
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("cylinder_target", "link7_1");
            task.add(std::move(stage));
        }

        RCLCPP_INFO(this->get_logger(), "Phase2: Task created with arc waypoints + fuzzy pose + OMPL");
        return task;
    }
};

#endif // PHASE2_NODE_HPP
