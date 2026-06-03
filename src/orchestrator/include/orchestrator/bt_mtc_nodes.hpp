#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <controller/phase0_node.hpp>
#include <controller/phase1_node.hpp>
#include <controller/phase2_node.hpp>
#include <controller/phase3_node.hpp>

#include <future>
#include <memory>
#include <chrono>
#include <sstream>
#include <array>
#include <vector>

namespace orchestrator {

/**
 * @brief MTC 执行结果
 */
struct MTCExecutionResult {
    bool success{false};
    std::string error_msg;
};

/**
 * @brief BT 节点基类 - 用于异步执行 MTC Phase 任务
 */
class MTCActionNode : public BT::StatefulActionNode {
public:
    MTCActionNode(const std::string& name, const BT::NodeConfiguration& config)
        : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { 
            BT::OutputPort<bool>("success"),
            BT::OutputPort<std::string>("error_msg")
        };
    }

    BT::NodeStatus onStart() override {
        future_ = std::async(std::launch::async, [this]() { return this->executePhase(); });
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
            return BT::NodeStatus::RUNNING;
        }

        const auto result = future_.get();
        setOutput("success", result.success);
        setOutput("error_msg", result.error_msg);

        return result.success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void onHalted() override {
    }

protected:
    virtual MTCExecutionResult executePhase() = 0;

    rclcpp::NodeOptions makePhaseNodeOptions(const rclcpp::Node::SharedPtr& source) const {
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true);

        std::vector<rclcpp::Parameter> overrides;
        const std::array<std::string, 5> required_params = {
            "robot_description",
            "robot_description_semantic",
            "robot_description_kinematics",
            "robot_description_planning",
            "planning_pipelines"};

        for (const auto& name : required_params) {
            rclcpp::Parameter parameter;
            if (source->get_parameter(name, parameter)) {
                overrides.push_back(parameter);
            }
        }

        options.parameter_overrides(overrides);
        return options;
    }

    static std::string uniqueNodeName(const std::string& prefix) {
        const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
        std::ostringstream oss;
        oss << prefix << "_" << now;
        return oss.str();
    }

    std::future<MTCExecutionResult> future_;
};

/**
 * @brief Phase 0 - Load/Align 阶段的 MTC 执行节点
 */
class ExecutePhase0 : public MTCActionNode {
public:
    ExecutePhase0(const std::string& name, const BT::NodeConfiguration& config, 
                  rclcpp::Node::SharedPtr node)
        : MTCActionNode(name, config), node_(node) {}

protected:
    MTCExecutionResult executePhase() override {
        RCLCPP_INFO(node_->get_logger(), "Starting Phase 0: Load/Align");

        try {
            auto options = makePhaseNodeOptions(node_);
            phase0_instance_ = std::make_shared<Phase0Node>(uniqueNodeName("phase0_node"), options);
            return {true, ""};
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Phase 0 failed: %s", e.what());
            return {false, e.what()};
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Phase0Node> phase0_instance_;
};

/**
 * @brief Phase 1 - Push 阶段的 MTC 执行节点
 */
class ExecutePhase1 : public MTCActionNode {
public:
    ExecutePhase1(const std::string& name, const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node)
        : MTCActionNode(name, config), node_(node) {}

protected:
    MTCExecutionResult executePhase() override {
        RCLCPP_INFO(node_->get_logger(), "Starting Phase 1: Push");

        try {
            auto options = makePhaseNodeOptions(node_);
            phase1_instance_ = std::make_shared<Phase1Node>(uniqueNodeName("phase1_node"), options);
            return {true, ""};
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Phase 1 failed: %s", e.what());
            return {false, e.what()};
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Phase1Node> phase1_instance_;
};

/**
 * @brief Phase 2 - Circular Rotation 阶段的 MTC 执行节点
 */
class ExecutePhase2 : public MTCActionNode {
public:
    ExecutePhase2(const std::string& name, const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node)
        : MTCActionNode(name, config), node_(node) {}

protected:
    MTCExecutionResult executePhase() override {
        RCLCPP_INFO(node_->get_logger(), "Starting Phase 2: Circular Rotation (-90 deg)");

        try {
            auto options = makePhaseNodeOptions(node_);
            phase2_instance_ = std::make_shared<Phase2Node>(uniqueNodeName("phase2_node"), options);
            return {true, ""};
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Phase 2 failed: %s", e.what());
            return {false, e.what()};
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Phase2Node> phase2_instance_;
};

/**
 * @brief Phase 3 - Y轴旋转 45° 阶段的 MTC 执行节点
 */
class ExecutePhase3 : public MTCActionNode {
public:
    ExecutePhase3(const std::string& name, const BT::NodeConfiguration& config,
                  rclcpp::Node::SharedPtr node)
        : MTCActionNode(name, config), node_(node) {}

protected:
    MTCExecutionResult executePhase() override {
        RCLCPP_INFO(node_->get_logger(), "Starting Phase 3: Y-Axis Rotation (+45 deg)");

        try {
            auto options = makePhaseNodeOptions(node_);
            phase3_instance_ = std::make_shared<Phase3Node>(uniqueNodeName("phase3_node"), options);
            return {true, ""};
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Phase 3 failed: %s", e.what());
            return {false, e.what()};
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<Phase3Node> phase3_instance_;
};

/**
 * @brief 发布当前阶段状态的节点
 */
class PublishPhaseNode : public BT::SyncActionNode {
public:
    PublishPhaseNode(const std::string& name, const BT::NodeConfiguration& config,
                     rclcpp::Node::SharedPtr node)
        : SyncActionNode(name, config), node_(node) {
        phase_pub_ = node_->create_publisher<std_msgs::msg::Int8>("orchestrator/phase", 10);
    }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("phase") };
    }

    BT::NodeStatus tick() override {
        int phase;
        if (getInput("phase", phase)) {
            std_msgs::msg::Int8 msg;
            msg.data = static_cast<int8_t>(phase);
            phase_pub_->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "Published phase: %d", phase);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr phase_pub_;
};

} // namespace orchestrator
