#pragma once
#include <chrono>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/task_constructor/stages.h"
#include "moveit/task_constructor/task.h"
#include "moveit/task_constructor/storage.h"
#include "moveit/task_constructor/solvers.h"
#include "moveit/robot_model/revolute_joint_model.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit_task_constructor_msgs/action/execute_task_solution.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "controller/fuzzy_pose_generator.hpp"

namespace mtc = moveit::task_constructor;

class MTC {
public:
    struct TaskTimingStats {
        long plan_ms{0};
        long execute_ms{0};
        long total_ms{0};
        bool success{false};
    };

    MTC(const rclcpp::NodeOptions& options, const std::string& node_name = "mtc_node")
    : node_(std::make_shared<rclcpp::Node>(node_name, options)) {}

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
        return node_->get_node_base_interface();
    }

    ~MTC() = default;

    bool doTask(){
        const auto total_start = std::chrono::steady_clock::now();
        last_task_timing_stats_ = {};

        task_ = create_task();
        task_.enableIntrospection(true);

        try{
            task_.init();
            task_.introspection().publishTaskDescription();
        }
        catch (mtc::InitStageException& e){
            last_task_timing_stats_.total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
            RCLCPP_ERROR_STREAM(LOGGER, e);
            return false;
        }

        const auto plan_start = std::chrono::steady_clock::now();
        if (!task_.plan(5)){
            last_task_timing_stats_.plan_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - plan_start).count();
            last_task_timing_stats_.total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return false;
        }
        last_task_timing_stats_.plan_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - plan_start).count();

        task_.introspection().publishTaskState();
        
        if (task_.solutions().empty()){
            last_task_timing_stats_.total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
            RCLCPP_ERROR_STREAM(LOGGER, "No solutions found");
            return false;
        }

        auto ranked_solutions = rankSolutionsByJointMotionCost();
        if (ranked_solutions.empty()) {
            last_task_timing_stats_.total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
            RCLCPP_ERROR_STREAM(LOGGER, "Failed to rank task solutions");
            return false;
        }

        const auto best_solution = ranked_solutions.front().first;
        const double best_cost = ranked_solutions.front().second;

        RCLCPP_INFO(LOGGER, "Selected best plan by custom joint-motion cost: %.6f (from %zu solutions)",
                    best_cost, ranked_solutions.size());
        
        task_.introspection().publishSolution(*best_solution);
        task_.introspection().publishAllSolutions(false);

        // Wait for the execute_task_solution action server provided by move_group + MTC capability
        if (!wait_for_execute_server(std::chrono::seconds(45))){
            last_task_timing_stats_.total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
            RCLCPP_ERROR(LOGGER, "Failed to find execute_task_solution action server");
            return false;
        }

        const auto execute_start = std::chrono::steady_clock::now();
        auto result = task_.execute(*best_solution);
        last_task_timing_stats_.execute_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - execute_start).count();
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
            last_task_timing_stats_.total_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            return false;
        }

        task_.introspection().publishTaskState();
        task_.introspection().publishSolution(*best_solution);
        task_.introspection().publishAllSolutions(false);

        last_task_timing_stats_.total_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - total_start).count();
        last_task_timing_stats_.success = true;
        return true;
    }

    const TaskTimingStats& getLastTaskTimingStats() const {
        return last_task_timing_stats_;
    }

    rclcpp::Node::SharedPtr getNodeSharedPtr(){
        return node_;
    }

    mtc::Task TaskInit(const std::string task_name){
        mtc::Task task;
        task.stages()->setName(task_name);
        task.loadRobotModel(MTC::getNodeSharedPtr());

        const auto& arm_group = "right_arm";
        const auto& hand_group = "right_gripper";
        const auto& hand_frame = "link7_1";

        task.setProperty("group", arm_group);        
        task.setProperty("hand_group", hand_group);
        task.setProperty("eef", hand_frame);
        
        task.setProperty("execute_action_name", std::string("/execute_task_solution"));
        
        return task;
    }

    virtual void setup_planning_scene() = 0;
    virtual mtc::Task create_task() = 0;

private:
    double computeTrajectoryJointDistanceCost(const robot_trajectory::RobotTrajectory& trajectory) const {
        const size_t waypoint_count = trajectory.getWayPointCount();
        if (waypoint_count < 2) {
            return 0.0;
        }

        const auto* joint_model_group = trajectory.getGroup();
        if (!joint_model_group) {
            return 0.0;
        }

        const auto& variable_names = joint_model_group->getVariableNames();
        if (variable_names.empty()) {
            return 0.0;
        }

        // 权重：x_rail_joint 占 0.65，其它关节均分 0.35
        size_t non_rail_count = 0;
        for (const auto& name : variable_names) {
            if (name != "x_rail_joint") {
                ++non_rail_count;
            }
        }

        const double x_rail_weight = 0.65;
        const double remaining_weight = 1.0 - x_rail_weight;
        const double non_rail_weight =
            non_rail_count > 0 ? (remaining_weight / static_cast<double>(non_rail_count)) : 0.0;

        std::vector<double> weights;
        weights.reserve(variable_names.size());
        for (const auto& name : variable_names) {
            weights.push_back(name == "x_rail_joint" ? x_rail_weight : non_rail_weight);
        }

        // 为每个变量准备归一化范围（range）和是否连续转动
        const auto& first_state = trajectory.getWayPoint(0);
        const auto* robot_model = first_state.getRobotModel().get();

        std::vector<double> ranges;
        std::vector<bool> is_continuous_revolute;
        ranges.reserve(variable_names.size());
        is_continuous_revolute.reserve(variable_names.size());

        for (const auto& var_name : variable_names) {
            double range = 1.0;  // fallback，避免除零
            bool continuous_rev = false;

            if (robot_model) {
                const auto& vb = robot_model->getVariableBounds(var_name);

                if (vb.position_bounded_) {
                    const double bounded_range = vb.max_position_ - vb.min_position_;
                    if (bounded_range > 1e-9) {
                        range = bounded_range;
                    }
                }

                if (const auto* jm = robot_model->getJointOfVariable(var_name)) {
                    if (jm->getType() == moveit::core::JointModel::REVOLUTE) {
                        const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(jm);
                        continuous_rev = revolute && revolute->isContinuous();
                    }

                    // 连续转动关节：用 2π 作为归一化范围
                    if (continuous_rev) {
                        range = 2.0 * M_PI;
                    }
                }
            }

            ranges.push_back(range);
            is_continuous_revolute.push_back(continuous_rev);
        }

        double total_cost = 0.0;
        for (size_t wp = 1; wp < waypoint_count; ++wp) {
            const auto& prev = trajectory.getWayPoint(wp - 1);
            const auto& curr = trajectory.getWayPoint(wp);

            for (size_t j = 0; j < variable_names.size(); ++j) {
                const auto& joint_name = variable_names[j];
                const double prev_q = prev.getVariablePosition(joint_name);
                const double curr_q = curr.getVariablePosition(joint_name);

                double delta = std::abs(curr_q - prev_q);

                // 连续转动关节按最短角距离处理（[-pi, pi]）
                if (is_continuous_revolute[j]) {
                    delta = std::abs(std::remainder(curr_q - prev_q, 2.0 * M_PI));
                }

                const double normalized_delta = delta / ranges[j];
                total_cost += weights[j] * normalized_delta;
            }
        }

        return total_cost;
    }

    double computeSolutionJointDistanceCost(const mtc::SolutionBase& solution) const {
        if (const auto* sub_trajectory = dynamic_cast<const mtc::SubTrajectory*>(&solution)) {
            const auto trajectory = sub_trajectory->trajectory();
            return trajectory ? computeTrajectoryJointDistanceCost(*trajectory) : 0.0;
        }

        if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
            double total_cost = 0.0;
            for (const auto* child : sequence->solutions()) {
                if (child) {
                    total_cost += computeSolutionJointDistanceCost(*child);
                }
            }
            return total_cost;
        }

        if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution)) {
            const auto* wrapped_solution = wrapped->wrapped();
            return wrapped_solution ? computeSolutionJointDistanceCost(*wrapped_solution) : 0.0;
        }

        return 0.0;
    }

    std::vector<std::pair<mtc::SolutionBaseConstPtr, double>> rankSolutionsByJointMotionCost() const {
        std::vector<std::pair<mtc::SolutionBaseConstPtr, double>> ranked;
        ranked.reserve(task_.solutions().size());

        for (const auto& solution : task_.solutions()) {
            ranked.emplace_back(solution, computeSolutionJointDistanceCost(*solution));
        }

        std::sort(ranked.begin(), ranked.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
        });

        return ranked;
    }

    bool wait_for_execute_server(std::chrono::seconds timeout){
        using ExecuteAction = moveit_task_constructor_msgs::action::ExecuteTaskSolution;
        std::array<std::string, 3> names = {"/execute_task_solution", "/move_group/execute_task_solution", "execute_task_solution"};

        const auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok()){
            for (const auto& name : names){
                auto client = rclcpp_action::create_client<ExecuteAction>(node_, name);
                if (client->wait_for_action_server(std::chrono::seconds(1))){
                    RCLCPP_INFO(LOGGER, "Found action server: %s", name.c_str());
                    return true;
                }
            }

            const auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed >= timeout){
                break;
            }

            RCLCPP_INFO(LOGGER, "Waiting for execute_task_solution action server...");
        }
        return false;
    }

    

    mtc::Task task_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger LOGGER = this->node_->get_logger();
    TaskTimingStats last_task_timing_stats_;
};
