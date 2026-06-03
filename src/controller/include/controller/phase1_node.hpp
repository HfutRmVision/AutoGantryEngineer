#ifndef PHASE1_NODE_HPP
#define PHASE1_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "controller/mtc.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

class Phase1Node : public rclcpp::Node, public MTC {
public:
    Phase1Node(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc"),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        RCLCPP_INFO(this->get_logger(), "Phase1: Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero, std::chrono::seconds(1));
                tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Phase1: TF available, starting task");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "Phase1: Waiting for transform: %s", e.what());
            }
        }
        doTask();
    }

private:
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void setup_planning_scene() override{}

    mtc::Task create_task() override{
        mtc::Task task = TaskInit("push");

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxAccelerationScalingFactor(0.1);
        sampling_planner->setMaxVelocityScalingFactor(0.1);
        sampling_planner->setProperty("planning_time", 5.0);

        mtc::Stage* current_state_ptr = nullptr;

        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            task.add(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("cylinder_target", "link7_1");
            current_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        {
            mtc::stages::Connect::GroupPlannerVector planners = {
                {"right_arm", sampling_planner}
            };
            auto stage = std::make_unique<mtc::stages::Connect>("sampling push connect", planners);
            stage->setTimeout(20.0);
            task.add(std::move(stage));
        }

        geometry_msgs::msg::PoseStamped goal_pose_world;
        {
            geometry_msgs::msg::TransformStamped ee_tf;
            geometry_msgs::msg::TransformStamped target_tf;
            try {
                ee_tf = tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero);
                target_tf = tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero);
            } catch (const std::exception & e) {
                RCLCPP_WARN(this->get_logger(), "Phase1: Could not get transform for push goal: %s", e.what());
                return task;
            }

            goal_pose_world.header.frame_id = "world";
            goal_pose_world.header.stamp = this->now();
            goal_pose_world.pose.position.x = ee_tf.transform.translation.x;
            goal_pose_world.pose.position.y = ee_tf.transform.translation.y;
            goal_pose_world.pose.position.z = ee_tf.transform.translation.z;
            goal_pose_world.pose.orientation = ee_tf.transform.rotation;

            tf2::Quaternion frame_q(
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w);
            tf2::Vector3 local_step(0.0, 0.1, 0.0);
            tf2::Vector3 world_step = tf2::quatRotate(frame_q, local_step);

            goal_pose_world.pose.position.x += world_step.x();
            goal_pose_world.pose.position.y += world_step.y();
            goal_pose_world.pose.position.z += world_step.z();
        }

        {
            auto generator = std::make_unique<FuzzyPoseGenerator>("fuzzy push goal");
            generator->setSampleCount(50);
            generator->setTolerance(0.001, 0.1);
            generator->setPose(goal_pose_world);
            generator->setMonitoredStage(current_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto stage = std::make_unique<mtc::stages::ComputeIK>("compute ik push goal", std::move(generator));
            stage->setMaxIKSolutions(10);
            stage->setMinSolutionDistance(0.1);
            stage->setIKFrame("link7_1");
            stage->setTargetPose(goal_pose_world);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setIgnoreCollisions(true);
            task.add(std::move(stage));
        }

        return task;
    }
};

#endif // PHASE1_NODE_HPP