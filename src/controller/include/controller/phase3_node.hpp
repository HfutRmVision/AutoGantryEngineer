#ifndef PHASE3_NODE_HPP
#define PHASE3_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "controller/mtc.hpp"
#include "controller/circular_path_generator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

/**
 * @brief Phase 3 - 以 target frame 推导当前 TF，并绕相对 Y 轴旋转 45°
 *
 * 几何定义：
 * 1) 先取 world -> cylinder_target_frame
 * 2) 在 target 局部坐标系下执行：y +377mm, z -61mm, Rx(-90deg)
 *    得到 world -> current_tf（虚拟当前位姿）
 * 3) 旋转轴相对 current_tf：平行于 y 轴，offset (x=0, z=-54mm)
 * 4) 末端绕该旋转轴旋转 +45deg
 *
 * 规划策略：
 * - Alternative A: Pilz CIRC（中间点约束）
 * - Alternative B: Waypoints + CartesianPath 分段近似圆弧
 */
class Phase3Node : public rclcpp::Node, public MTC {
public:
    Phase3Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc"),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
    {
        RCLCPP_INFO(this->get_logger(), "Phase3: Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7_1",
                    tf2::TimePointZero, std::chrono::seconds(1));
                tf_buffer_.lookupTransform("world", "cylinder_target_frame",
                    tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Phase3: TF available");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", e.what());
            }
        }

        if (!doTask()) {
            RCLCPP_WARN(this->get_logger(), "Phase3: Task failed, executing fallback (home)");
            doFallback();
        }
    }

private:
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    static constexpr double TARGET_Y_OFFSET = 0.377;         // +377mm
    static constexpr double TARGET_Z_OFFSET = -0.061;        // -61mm
    static constexpr double TARGET_X_ROTATION = -M_PI_2;     // -90deg
    static constexpr double AXIS_X_OFFSET = 0.0;
    static constexpr double AXIS_Z_OFFSET = -0.054;          // -54mm
    static constexpr double ROTATION_ANGLE = M_PI / 4.0;     // +45deg
    static constexpr int WAYPOINT_COUNT = 24;

    struct RotationAxisGeometry {
        tf2::Vector3 origin_world;
        tf2::Vector3 direction_world;
    };

    geometry_msgs::msg::PoseStamped getCurrentLink7Pose()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = this->now();

        try {
            const auto ee_tf = tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero);
            pose.pose.position.x = ee_tf.transform.translation.x;
            pose.pose.position.y = ee_tf.transform.translation.y;
            pose.pose.position.z = ee_tf.transform.translation.z;
            pose.pose.orientation = ee_tf.transform.rotation;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Phase3: Could not get current link7 pose: %s", e.what());
        }

        return pose;
    }

    geometry_msgs::msg::TransformStamped deriveCurrentTF(
        const geometry_msgs::msg::TransformStamped& target_tf) const
    {
        tf2::Transform world_to_target;
        tf2::fromMsg(target_tf.transform, world_to_target);

        tf2::Transform target_to_current;
        tf2::Quaternion q_local;
        q_local.setRPY(TARGET_X_ROTATION, 0.0, 0.0);
        q_local.normalize();
        target_to_current.setOrigin(tf2::Vector3(0.0, TARGET_Y_OFFSET, TARGET_Z_OFFSET));
        target_to_current.setRotation(q_local);

        tf2::Transform world_to_current = world_to_target * target_to_current;

        geometry_msgs::msg::TransformStamped current_tf;
        current_tf.header.frame_id = "world";
        current_tf.header.stamp = this->now();
        current_tf.child_frame_id = "phase3_current_tf";
        current_tf.transform = tf2::toMsg(world_to_current);
        return current_tf;
    }

    geometry_msgs::msg::PoseStamped toPoseStamped(
        const geometry_msgs::msg::TransformStamped& tf_msg) const
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = tf_msg.header;
        pose.pose.position.x = tf_msg.transform.translation.x;
        pose.pose.position.y = tf_msg.transform.translation.y;
        pose.pose.position.z = tf_msg.transform.translation.z;
        pose.pose.orientation = tf_msg.transform.rotation;
        return pose;
    }

    RotationAxisGeometry computeRotationAxis(
        const geometry_msgs::msg::TransformStamped& current_tf) const
    {
        tf2::Quaternion q_current(
            current_tf.transform.rotation.x,
            current_tf.transform.rotation.y,
            current_tf.transform.rotation.z,
            current_tf.transform.rotation.w);
        tf2::Vector3 current_origin(
            current_tf.transform.translation.x,
            current_tf.transform.translation.y,
            current_tf.transform.translation.z);

        tf2::Vector3 axis_origin_local(AXIS_X_OFFSET, 0.0, AXIS_Z_OFFSET);
        tf2::Vector3 axis_origin_world = current_origin + tf2::quatRotate(q_current, axis_origin_local);

        tf2::Vector3 axis_dir_local(0.0, 1.0, 0.0);
        tf2::Vector3 axis_dir_world = tf2::quatRotate(q_current, axis_dir_local);
        axis_dir_world.normalize();

        return RotationAxisGeometry{axis_origin_world, axis_dir_world};
    }

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

        tf2::Vector3 r = point_world - axis_origin_world;
        const double cos_a = std::cos(angle);
        const double sin_a = std::sin(angle);
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

    moveit_msgs::msg::Constraints buildInterimConstraint(
        const geometry_msgs::msg::PoseStamped& interim_pose) const
    {
        moveit_msgs::msg::Constraints constraints;
        constraints.name = "phase3_pilz_interim";

        moveit_msgs::msg::PositionConstraint pos_constraint;
        pos_constraint.header.frame_id = "world";
        pos_constraint.link_name = "link7_1";
        pos_constraint.weight = 1.0;

        shape_msgs::msg::SolidPrimitive region;
        region.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        region.dimensions = {0.005};
        pos_constraint.constraint_region.primitives.push_back(region);

        geometry_msgs::msg::Pose interim_point;
        interim_point.position = interim_pose.pose.position;
        interim_point.orientation.w = 1.0;
        pos_constraint.constraint_region.primitive_poses.push_back(interim_point);

        constraints.position_constraints.push_back(pos_constraint);
        return constraints;
    }

    void doFallback()
    {
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            MTC::getNodeSharedPtr(), "right_arm");
        move_group->setMaxVelocityScalingFactor(0.1);
        move_group->setMaxAccelerationScalingFactor(0.1);
        move_group->setPlanningTime(15.0);
        move_group->setNumPlanningAttempts(5);

        constexpr int max_retry = 5;
        bool moved_home = false;
        for (int attempt = 1; attempt <= max_retry && rclcpp::ok(); ++attempt) {
            move_group->setStartStateToCurrentState();
            move_group->setNamedTarget("home_gr");

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_WARN(this->get_logger(),
                    "Phase3 fallback: planning failed (attempt %d/%d)", attempt, max_retry);
                continue;
            }

            RCLCPP_INFO(this->get_logger(),
                "Phase3 fallback: moving to home_gr (attempt %d/%d)", attempt, max_retry);
            if (move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                moved_home = true;
                break;
            }

            RCLCPP_WARN(this->get_logger(),
                "Phase3 fallback: execution failed (attempt %d/%d)", attempt, max_retry);
        }

        if (!moved_home) {
            RCLCPP_ERROR(this->get_logger(),
                "Phase3 fallback: failed to reach home_gr after %d attempts", max_retry);
        }
    }

    void setup_planning_scene() override {}

    mtc::Task create_task() override
    {
        mtc::Task task = TaskInit("phase3_axis_rotation");

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxVelocityScalingFactor(0.1);
        sampling_planner->setMaxAccelerationScalingFactor(0.1);
        sampling_planner->setProperty("planning_time", 10.0);

        auto pilz_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        pilz_planner->setPlannerId("pilz_industrial_motion_planner", "CIRC");
        pilz_planner->setProperty("planning_pipeline", std::string("pilz_industrial_motion_planner"));
        pilz_planner->setProperty("planning_time", 10.0);
        pilz_planner->setMaxVelocityScalingFactor(0.1);
        pilz_planner->setMaxAccelerationScalingFactor(0.1);

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(0.1);
        cartesian_planner->setMaxAccelerationScalingFactor(0.1);
        cartesian_planner->setStepSize(0.0015);
        cartesian_planner->setMinFraction(0.95);
        cartesian_planner->setIKFrame("link7_1");

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

        geometry_msgs::msg::PoseStamped reload_goal_pose = getCurrentLink7Pose();
        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator for reloading");
            generator->setSampleCount(30);
            generator->setTolerance(0.01, 0.2);
            generator->setPose(reload_goal_pose);
            generator->setMonitoredStage(detach_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});

            auto stage = std::make_unique<mtc::stages::ComputeIK>("compute ik for reloading", std::move(generator));
            stage->setMaxIKSolutions(2);
            stage->setMinSolutionDistance(2.0);
            stage->setIKFrame("link7_1");
            stage->setTargetPose(reload_goal_pose);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
            stage->setIgnoreCollisions(false);
            task.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->allowCollisions("cylinder_target", "right_arm", true);
            stage->attachObject("cylinder_target", "link7_1");
            task.add(std::move(stage));
        }

        RCLCPP_INFO(this->get_logger(),
            "Phase3: reload pose from current link7 (%.4f, %.4f, %.4f)",
            reload_goal_pose.pose.position.x,
            reload_goal_pose.pose.position.y,
            reload_goal_pose.pose.position.z);

        geometry_msgs::msg::TransformStamped target_tf;
        try {
            target_tf = tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero);
        } catch (const tf2::TransformException& e) {
            RCLCPP_ERROR(this->get_logger(), "Phase3: target tf lookup failed: %s", e.what());
            return task;
        }

        const auto current_tf = deriveCurrentTF(target_tf);
        const auto current_pose = toPoseStamped(current_tf);
        const auto axis_geometry = computeRotationAxis(current_tf);
        const auto interim_pose = rotatePoseAroundAxis(
            current_pose, axis_geometry.origin_world, axis_geometry.direction_world, ROTATION_ANGLE / 2.0);
        const auto goal_pose = rotatePoseAroundAxis(
            current_pose, axis_geometry.origin_world, axis_geometry.direction_world, ROTATION_ANGLE);

        CircularPathGenerator gen;
        gen.setRotationAxisOffset(AXIS_X_OFFSET, 0.0, AXIS_Z_OFFSET);
        gen.setRotationAxisDirection(1);
        gen.setRotationAngle(ROTATION_ANGLE);
        gen.setWaypointCount(WAYPOINT_COUNT);

        auto waypoints = gen.generateWaypoints(current_pose, current_tf);
        const double radius = gen.getRotationRadius(current_pose, current_tf);

        RCLCPP_INFO(this->get_logger(),
            "Phase3: derived current_tf and axis computed (radius: %.3f m, angle: %.1f deg, waypoints: %zu)",
            radius, ROTATION_ANGLE * 180.0 / M_PI, static_cast<int>(waypoints.size()));
        RCLCPP_INFO(this->get_logger(),
            "Phase3: current_tf pos (%.4f, %.4f, %.4f), axis origin (%.4f, %.4f, %.4f), axis dir (%.4f, %.4f, %.4f)",
            current_tf.transform.translation.x,
            current_tf.transform.translation.y,
            current_tf.transform.translation.z,
            axis_geometry.origin_world.x(),
            axis_geometry.origin_world.y(),
            axis_geometry.origin_world.z(),
            axis_geometry.direction_world.x(),
            axis_geometry.direction_world.y(),
            axis_geometry.direction_world.z());

        if (waypoints.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Phase3: Not enough waypoints generated!");
            return task;
        }

        auto alternatives = std::make_unique<mtc::Alternatives>("phase3_planning_alternatives");

        {
            auto pilz_branch = std::make_unique<mtc::SerialContainer>("pilz_circ_branch");
            auto pilz_stage = std::make_unique<mtc::stages::MoveTo>("pilz_circ_rotate_45deg", pilz_planner);
            pilz_stage->setGroup("right_arm");
            pilz_stage->setIKFrame("link7_1");
            pilz_stage->setTimeout(8.0);
            pilz_stage->setGoal(goal_pose);
            pilz_stage->setPathConstraints(buildInterimConstraint(interim_pose));
            pilz_branch->add(std::move(pilz_stage));
            alternatives->add(std::move(pilz_branch));
        }

        {
            auto cartesian_branch = std::make_unique<mtc::SerialContainer>("waypoints_cartesian_branch");
            for (size_t i = 1; i < waypoints.size(); ++i) {
                auto stage = std::make_unique<mtc::stages::MoveTo>(
                    "cartesian_arc_step_" + std::to_string(i), cartesian_planner);
                stage->setGroup("right_arm");
                stage->setIKFrame("link7_1");
                stage->setTimeout(2.5);
                stage->setGoal(waypoints[i]);
                cartesian_branch->add(std::move(stage));
            }
            alternatives->add(std::move(cartesian_branch));
        }

        task.add(std::move(alternatives));

        RCLCPP_INFO(this->get_logger(),
            "Phase3: Task created with Alternatives [Pilz CIRC + Waypoints Cartesian], cartesian stages: %zu",
            waypoints.size() - 1);
        return task;
    }
};

#endif // PHASE3_NODE_HPP
