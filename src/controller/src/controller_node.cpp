#include "rclcpp/rclcpp.hpp"
#include "controller/phase0_node.hpp"
#include "controller/phase1_node.hpp"
#include "controller/phase_collection_node.hpp"
#include "controller/phase_test_node.hpp"

#include <array>
#include <vector>

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto options = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    auto param_source_node = std::make_shared<rclcpp::Node>("controller_node", options);

    std::vector<rclcpp::Parameter> overrides;
    const std::array<std::string, 7> required_params = {
        "robot_description",
        "robot_description_semantic",
        "robot_description_kinematics",
        "robot_description_planning",
        "planning_pipelines",
        "joint_limits",
        "use_sim_time"
    };

    for (const auto& name : required_params) {
        rclcpp::Parameter parameter;
        if (param_source_node->get_parameter(name, parameter)) {
            overrides.push_back(parameter);
        }
    }

    auto phase_options = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
        .parameter_overrides(overrides);

    auto phase_test_node = std::make_shared<PhaseCollectionNode>("phase_collection_node", phase_options);
    //auto phase1_node = std::make_shared<Phase1Node>("phase1_node", options);

    executor.add_node(phase_test_node);
    //executor.add_node(phase1_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}