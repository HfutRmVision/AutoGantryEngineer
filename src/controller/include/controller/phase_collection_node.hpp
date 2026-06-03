#ifndef PHASE_COLLECTION_NODE_HPP
#define PHASE_COLLECTION_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "controller/mtc.hpp"
#include "controller/fuzzy_pose_generator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include <algorithm>

class PhaseCollectionNode : public rclcpp::Node, public MTC {
public:
    PhaseCollectionNode(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc"),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        RCLCPP_INFO(this->get_logger(), "PhaseCollection: Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero, std::chrono::seconds(1));
                tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "PhaseCollection: TF available, starting combined task");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform: %s", e.what());
            }
        }

        const bool success = doTask();
        const auto& timing = getLastTaskTimingStats();

        RCLCPP_INFO(
            this->get_logger(),
            "\033[1;32m"
            "PhaseCollection Performance\n"
            "  plan    : %ld ms\n"
            "  execute : %ld ms\n"
            "  total   : %ld ms\n"
            "  success : %s"
            "\033[0m",
            timing.plan_ms,
            timing.execute_ms,
            timing.total_ms,
            success ? "true" : "false");
    }

private:
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    static constexpr double PHASE2_P_AXIS_Y_OFFSET = 0.108;
    static constexpr double PHASE2_P_AXIS_Z_OFFSET = -0.169;
    static constexpr double PHASE2_ROTATION_ANGLE  = -M_PI_2;  // -90°
    static constexpr double CYLINDER_BODY_Z_OFFSET_FROM_LINK7 = 0.05;

    geometry_msgs::msg::PoseStamped rotateEEAroundAxis(
        const geometry_msgs::msg::PoseStamped& ee_world,
        const tf2::Vector3& axis_origin_world,
        const tf2::Vector3& axis_dir_world,
        double angle)
    {
        tf2::Vector3 ee_pos(ee_world.pose.position.x,
                            ee_world.pose.position.y,
                            ee_world.pose.position.z);
        tf2::Quaternion ee_ori(ee_world.pose.orientation.x,
                               ee_world.pose.orientation.y,
                               ee_world.pose.orientation.z,
                               ee_world.pose.orientation.w);

        tf2::Vector3 r = ee_pos - axis_origin_world;
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        tf2::Vector3 r_rot = r * cos_a
                            + axis_dir_world.cross(r) * sin_a
                            + axis_dir_world * axis_dir_world.dot(r) * (1.0 - cos_a);
        tf2::Vector3 new_pos = axis_origin_world + r_rot;

        tf2::Quaternion rot_q;
        rot_q.setRotation(axis_dir_world, angle);
        tf2::Quaternion new_ori = rot_q * ee_ori;
        new_ori.normalize();

        geometry_msgs::msg::PoseStamped result;
        result.header.frame_id = "world";
        result.pose.position.x = new_pos.x();
        result.pose.position.y = new_pos.y();
        result.pose.position.z = new_pos.z();
        result.pose.orientation = tf2::toMsg(new_ori);
        return result;
    }

    geometry_msgs::msg::PoseStamped deriveAttachedCylinderBodyPose(
        const geometry_msgs::msg::PoseStamped& ee_world) const
    {
        tf2::Quaternion ee_q(
            ee_world.pose.orientation.x,
            ee_world.pose.orientation.y,
            ee_world.pose.orientation.z,
            ee_world.pose.orientation.w);
        tf2::Vector3 body_offset_local(0.0, 0.0, CYLINDER_BODY_Z_OFFSET_FROM_LINK7);
        tf2::Vector3 body_offset_world = tf2::quatRotate(ee_q, body_offset_local);

        geometry_msgs::msg::PoseStamped cylinder_body_pose = ee_world;
        cylinder_body_pose.pose.position.x += body_offset_world.x();
        cylinder_body_pose.pose.position.y += body_offset_world.y();
        cylinder_body_pose.pose.position.z += body_offset_world.z();
        return cylinder_body_pose;
    }

    geometry_msgs::msg::TransformStamped get_tf() {
        try {
            return tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero);
        } catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", e.what());
            return geometry_msgs::msg::TransformStamped();
        }
    }

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
            RCLCPP_WARN(this->get_logger(), "Could not get current link7 pose: %s", e.what());
        }
        return pose;
    }

    void setup_planning_scene() override {}

        mtc::Task create_task() override {
        mtc::Task task = TaskInit("phase_collection_task");

        auto phase0_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        phase0_sampling_planner->setMaxAccelerationScalingFactor(0.6);
        phase0_sampling_planner->setMaxVelocityScalingFactor(0.6);
        phase0_sampling_planner->setProperty("planning_time", 5.0);

        auto phase1_sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        phase1_sampling_planner->setMaxAccelerationScalingFactor(0.15);
        phase1_sampling_planner->setMaxVelocityScalingFactor(0.15);
        phase1_sampling_planner->setProperty("planning_time", 5.0);

        task.properties().set("group", "right_arm");
        task.properties().set("ik_frame", "link7_1");

        mtc::Stage* current_state_ptr = nullptr;
        mtc::Stage* preload_state_ptr = nullptr;
        mtc::Stage* phase1_attach_state_ptr = nullptr;

        // ===== Phase0: load/align =====
        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            current_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        {
            mtc::stages::Connect::GroupPlannerVector planners = { {"right_arm", phase0_sampling_planner} };
            auto stage = std::make_unique<mtc::stages::Connect>("move to preload", planners);
            stage->setTimeout(10.0);
            task.add(std::move(stage));
        }

        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator for preloading");
            generator->setSampleCount(500);
            generator->setTolerance(0.02, 0.3);

            geometry_msgs::msg::TransformStamped target_tf = get_tf();
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "world";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = target_tf.transform.translation.x;
            goal_pose.pose.position.y = target_tf.transform.translation.y;
            goal_pose.pose.position.z = target_tf.transform.translation.z;
            goal_pose.pose.orientation = target_tf.transform.rotation;

            tf2::Quaternion q(
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w);
            tf2::Vector3 z_offset(0.0, 0.0, 0.03);
            tf2::Vector3 rotated_offset = tf2::quatRotate(q, z_offset);

            goal_pose.pose.position.x += rotated_offset.x();
            goal_pose.pose.position.y += rotated_offset.y();
            goal_pose.pose.position.z += rotated_offset.z();
            goal_pose.pose.orientation = target_tf.transform.rotation;

            generator->setPose(goal_pose);
            generator->setMonitoredStage(current_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik for preloading", std::move(generator));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("link7_1");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            wrapper->setIgnoreCollisions(true);
            preload_state_ptr = wrapper.get();
            task.add(std::move(wrapper));
        }

        {
            mtc::stages::Connect::GroupPlannerVector planners = { {"right_arm", phase0_sampling_planner} };
            auto stage = std::make_unique<mtc::stages::Connect>("move to load", planners);
            stage->setTimeout(10.0);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            moveit_msgs::msg::Constraints path_constraints;
            geometry_msgs::msg::TransformStamped target_tf = get_tf();

            moveit_msgs::msg::OrientationConstraint ori_constraint;
            ori_constraint.link_name = "link7_1";
            ori_constraint.header.frame_id = "world";
            ori_constraint.orientation = target_tf.transform.rotation;
            ori_constraint.absolute_x_axis_tolerance = 0.2;
            ori_constraint.absolute_y_axis_tolerance = 0.2;
            ori_constraint.absolute_z_axis_tolerance = 3.14;

            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.link_name = "link7_1";
            pos_constraint.header.frame_id = "world";
            shape_msgs::msg::SolidPrimitive bounding_region;
            bounding_region.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            bounding_region.dimensions = {0.12, 0.02};
            pos_constraint.constraint_region.primitives.push_back(bounding_region);

            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "world";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = target_tf.transform.translation.x;
            goal_pose.pose.position.y = target_tf.transform.translation.y;
            goal_pose.pose.position.z = target_tf.transform.translation.z;
            goal_pose.pose.orientation = target_tf.transform.rotation;
            pos_constraint.constraint_region.primitive_poses.push_back(goal_pose.pose);

            path_constraints.orientation_constraints.push_back(ori_constraint);
            path_constraints.position_constraints.push_back(pos_constraint);
            stage->setPathConstraints(path_constraints);
            task.add(std::move(stage));
        }

        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy pose generator for loading");
            generator->setSampleCount(50);
            generator->setTolerance(0.005, 0.2);
            generator->setMonitoredStage(preload_state_ptr);

            geometry_msgs::msg::TransformStamped target_tf = get_tf();
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.frame_id = "world";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = target_tf.transform.translation.x;
            goal_pose.pose.position.y = target_tf.transform.translation.y;
            goal_pose.pose.position.z = target_tf.transform.translation.z;
            goal_pose.pose.orientation = target_tf.transform.rotation;

            tf2::Quaternion q(
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w);
            tf2::Vector3 z_offset(0.0, 0.0, -0.05);
            tf2::Vector3 rotated_offset = tf2::quatRotate(q, z_offset);

            goal_pose.pose.position.x += rotated_offset.x();
            goal_pose.pose.position.y += rotated_offset.y();
            goal_pose.pose.position.z += rotated_offset.z();
            goal_pose.pose.orientation = target_tf.transform.rotation;

            generator->setPose(goal_pose);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik for loading", std::move(generator));
            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("link7_1");
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            wrapper->setIgnoreCollisions(false);
            task.add(std::move(wrapper));
        }

        // ===== Phase1: attach + sampling push + fuzzy IK =====
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("cylinder_target", "link7_1");
            phase1_attach_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        {
            mtc::stages::Connect::GroupPlannerVector planners = {
                {"right_arm", phase1_sampling_planner}
            };
            auto stage = std::make_unique<mtc::stages::Connect>("sampling push connect", planners);
            stage->setTimeout(20.0);
            task.add(std::move(stage));
        }

        geometry_msgs::msg::PoseStamped phase1_goal_pose_world;
        {
            geometry_msgs::msg::TransformStamped ee_tf;
            geometry_msgs::msg::TransformStamped target_tf;
            try {
                ee_tf = tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero);
                target_tf = tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero);
            } catch (const std::exception & e) {
                RCLCPP_WARN(this->get_logger(), "PhaseCollection Phase1: Could not get transform for push goal: %s", e.what());
                return task;
            }

            phase1_goal_pose_world.header.frame_id = "world";
            phase1_goal_pose_world.header.stamp = this->now();
            phase1_goal_pose_world.pose.position.x = ee_tf.transform.translation.x;
            phase1_goal_pose_world.pose.position.y = ee_tf.transform.translation.y;
            phase1_goal_pose_world.pose.position.z = ee_tf.transform.translation.z;
            phase1_goal_pose_world.pose.orientation = ee_tf.transform.rotation;

            tf2::Quaternion frame_q(
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w);
            tf2::Vector3 local_step(0.0, 0.1, 0.0);
            tf2::Vector3 world_step = tf2::quatRotate(frame_q, local_step);

            phase1_goal_pose_world.pose.position.x += world_step.x();
            phase1_goal_pose_world.pose.position.y += world_step.y();
            phase1_goal_pose_world.pose.position.z += world_step.z();
        }

        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy push goal");
            generator->setSampleCount(50);
            generator->setTolerance(0.001, 0.1);
            generator->setPose(phase1_goal_pose_world);
            generator->setMonitoredStage(phase1_attach_state_ptr ? phase1_attach_state_ptr : current_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto stage = std::make_unique<mtc::stages::ComputeIK>("compute ik push goal", std::move(generator));
            stage->setMaxIKSolutions(10);
            stage->setMinSolutionDistance(0.1);
            stage->setIKFrame("link7_1");
            stage->setTargetPose(phase1_goal_pose_world);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setIgnoreCollisions(true);
            task.add(std::move(stage));
        }

        /*
        // ===== Phase2: circular rotation around P axis (-90 deg) =====
        {
            geometry_msgs::msg::PoseStamped phase2_current_pose = phase1_goal_pose_world;
            geometry_msgs::msg::PoseStamped attached_cylinder_body_pose = deriveAttachedCylinderBodyPose(phase2_current_pose);

            tf2::Quaternion cylinder_body_q(
                attached_cylinder_body_pose.pose.orientation.x,
                attached_cylinder_body_pose.pose.orientation.y,
                attached_cylinder_body_pose.pose.orientation.z,
                attached_cylinder_body_pose.pose.orientation.w);
            tf2::Vector3 cylinder_body_origin(
                attached_cylinder_body_pose.pose.position.x,
                attached_cylinder_body_pose.pose.position.y,
                attached_cylinder_body_pose.pose.position.z);

            tf2::Vector3 axis_origin_local(0.0, PHASE2_P_AXIS_Y_OFFSET, PHASE2_P_AXIS_Z_OFFSET);
            tf2::Vector3 axis_origin_world = cylinder_body_origin + tf2::quatRotate(cylinder_body_q, axis_origin_local);

            tf2::Vector3 axis_dir_local(1.0, 0.0, 0.0);
            tf2::Vector3 axis_dir_world = tf2::quatRotate(cylinder_body_q, axis_dir_local);
            axis_dir_world.normalize();

            auto interim_pose = rotateEEAroundAxis(
                phase2_current_pose, axis_origin_world, axis_dir_world, PHASE2_ROTATION_ANGLE / 2.0);
            auto goal_pose = rotateEEAroundAxis(
                phase2_current_pose, axis_origin_world, axis_dir_world, PHASE2_ROTATION_ANGLE);

            moveit_msgs::msg::Constraints circ_constraints;
            circ_constraints.name = "phase2_interim";

            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.header.frame_id = "world";
            pos_constraint.link_name = "link7_1";

            shape_msgs::msg::SolidPrimitive region;
            region.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            region.dimensions = {0.01};
            pos_constraint.constraint_region.primitives.push_back(region);

            geometry_msgs::msg::Pose interim_point;
            interim_point.position = interim_pose.pose.position;
            interim_point.orientation.w = 1.0;
            pos_constraint.constraint_region.primitive_poses.push_back(interim_point);
            circ_constraints.position_constraints.push_back(pos_constraint);

            auto stage = std::make_unique<mtc::stages::MoveTo>("phase2_circular_rotation", circ_planner);
            stage->setGroup("right_arm");
            stage->setGoal(goal_pose);
            stage->setIKFrame("link7_1");
            stage->setPathConstraints(circ_constraints);
            task.add(std::move(stage));
        }
        */

        // detach at end
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject("cylinder_target", "link7_1");
            task.add(std::move(stage));
        }

        return task;
    }
};

#endif // PHASE_COLLECTION_NODE_HPP
