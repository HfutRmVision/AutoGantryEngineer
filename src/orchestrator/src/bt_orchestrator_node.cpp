#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "rclcpp/rclcpp.hpp"
#include "orchestrator/bt_mtc_nodes.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class BTOrchestratorNode : public rclcpp::Node {
public:
    BTOrchestratorNode()
        : Node(
              "bt_orchestrator_node",
              rclcpp::NodeOptions()
                  .allow_undeclared_parameters(true)
                  .automatically_declare_parameters_from_overrides(true)) {
        if (!this->has_parameter("bt_xml_file")) {
            this->declare_parameter<std::string>("bt_xml_file", "");
        }
        if (!this->has_parameter("tick_rate_ms")) {
            this->declare_parameter<int>("tick_rate_ms", 100);
        }
    }

    bool initialize() {
        auto xml_file = this->get_parameter("bt_xml_file").as_string();
        const int tick_rate = this->get_parameter("tick_rate_ms").as_int();

        if (xml_file.empty()) {
            const auto pkg_path = ament_index_cpp::get_package_share_directory("orchestrator");
            xml_file = pkg_path + "/config/gantry_task.xml";
        }

        factory_ = std::make_unique<BT::BehaviorTreeFactory>();

        auto node_ptr = this->shared_from_this();

        factory_->registerBuilder<orchestrator::ExecutePhase0>(
            "ExecutePhase0",
            [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<orchestrator::ExecutePhase0>(name, config, node_ptr);
            });

        factory_->registerBuilder<orchestrator::ExecutePhase1>(
            "ExecutePhase1",
            [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<orchestrator::ExecutePhase1>(name, config, node_ptr);
            });

        factory_->registerBuilder<orchestrator::ExecutePhase2>(
            "ExecutePhase2",
            [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<orchestrator::ExecutePhase2>(name, config, node_ptr);
            });

        factory_->registerBuilder<orchestrator::ExecutePhase3>(
            "ExecutePhase3",
            [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<orchestrator::ExecutePhase3>(name, config, node_ptr);
            });

        factory_->registerBuilder<orchestrator::PublishPhaseNode>(
            "PublishPhase",
            [node_ptr](const std::string& name, const BT::NodeConfiguration& config) {
                return std::make_unique<orchestrator::PublishPhaseNode>(name, config, node_ptr);
            });

        try {
            tree_ = factory_->createTreeFromFile(xml_file);
            tree_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "BehaviorTree loaded: %s", xml_file.c_str());
        } catch (const std::exception& e) {
            tree_initialized_ = false;
            RCLCPP_ERROR(this->get_logger(), "Failed to load BehaviorTree: %s", e.what());
            return false;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(tick_rate),
            std::bind(&BTOrchestratorNode::tick, this));

        RCLCPP_INFO(this->get_logger(), "BT orchestrator started, tick rate=%d ms", tick_rate);
        return true;
    }

private:
    void tick() {
        if (!tree_initialized_) {
            return;
        }

        BT::NodeStatus status = BT::NodeStatus::FAILURE;
        try {
            status = tree_.tickOnce();
        } catch (const BT::NodeExecutionError& e) {
            RCLCPP_ERROR(this->get_logger(), "BehaviorTree execution error: %s", e.what());
            timer_->cancel();
            return;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected exception during BT tick: %s", e.what());
            timer_->cancel();
            return;
        }

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Task completed successfully.");
            timer_->cancel();
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Task failed.");
            timer_->cancel();
        }
    }

    std::unique_ptr<BT::BehaviorTreeFactory> factory_;
    bool tree_initialized_ = false;
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BTOrchestratorNode>();

    if (!node->initialize()) {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
