#ifndef PHASE_TEST_NODE_HPP
#define PHASE_TEST_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "controller/mtc.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

class PhaseTestNode : public rclcpp::Node, public MTC {
public:
    PhaseTestNode(const std::string & name, const rclcpp::NodeOptions & options)
    : rclcpp::Node(name, options), MTC(options, name + "_mtc"),
      tf_buffer_{this->get_clock()},
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
        RCLCPP_INFO(this->get_logger(), "PhaseTest: Waiting for TF...");
        while (rclcpp::ok()) {
            try {
                tf_buffer_.lookupTransform("world", "link7_1", tf2::TimePointZero, std::chrono::seconds(1));
                tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero, std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "PhaseTest: TF available, starting test task");
                break;
            } catch (const tf2::TransformException & e) {
                RCLCPP_WARN(this->get_logger(), "PhaseTest: Waiting for transform: %s", e.what());
            }
        }
        doTask();
    }

private:
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    static constexpr unsigned int MAX_PLANNING_ATTEMPTS = 20;

    geometry_msgs::msg::TransformStamped get_tf() {
        try {
            return tf_buffer_.lookupTransform("world", "cylinder_target_frame", tf2::TimePointZero);
        } catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(), "PhaseTest: Could not get transform: %s", e.what());
            return geometry_msgs::msg::TransformStamped();
        }
    }

    geometry_msgs::msg::PoseStamped build_phase0_final_goal_pose() {
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

        return goal_pose;
    }

    void setup_planning_scene() override {}

    mtc::Task create_task() override {
        mtc::Task task = TaskInit("phase_test");

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(MTC::getNodeSharedPtr());
        sampling_planner->setMaxAccelerationScalingFactor(0.2);
        sampling_planner->setMaxVelocityScalingFactor(0.2);
        sampling_planner->setProperty("planning_time", 10.0);
        sampling_planner->setProperty("num_planning_attempts", MAX_PLANNING_ATTEMPTS);

        task.properties().set("group", "right_arm");
        task.properties().set("ik_frame", "link7_1");

        mtc::Stage* current_state_ptr = nullptr;
        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
            current_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        {
            mtc::stages::Connect::GroupPlannerVector planners = {
                {"right_arm", sampling_planner}
            };
            auto stage = std::make_unique<mtc::stages::Connect>("connect current to goal", planners);
            stage->setTimeout(20.0);
            task.add(std::move(stage));
        }

        {
            geometry_msgs::msg::PoseStamped goal_pose = build_phase0_final_goal_pose();

            auto generator = std::make_unique<FuzzyPoseGenerator>("goal state");
            generator->setSampleCount(1);
            generator->setTolerance(0.0, 0.0);
            generator->setPose(goal_pose);
            generator->setMonitoredStage(current_state_ptr);
            generator->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("compute ik for goal state", std::move(generator));
            wrapper->setMaxIKSolutions(1);
            wrapper->setMinSolutionDistance(0.1);
            wrapper->setIKFrame("link7_1");
            wrapper->setTargetPose(goal_pose);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            wrapper->setIgnoreCollisions(false);
            task.add(std::move(wrapper));
        }

        return task;
    }
};

#endif // PHASE_TEST_NODE_HPP
