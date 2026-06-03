#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

#include <Eigen/Geometry>
#include <memory>
#include <vector>

class RecoverHomeNode : public rclcpp::Node
{
public:
    explicit RecoverHomeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("recover_home_node", options)
    {
        z_threshold_ = declare_parameter<double>("recover_z_threshold", 0.045);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&RecoverHomeNode::on_timer, this));

        RCLCPP_INFO(get_logger(), "RecoverHomeNode ready, auto-recover enabled");
    }

    void initialize_move_group()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "right_arm");
        RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized for group: right_arm");
    }

private:
    void on_timer()
    {
        if (executing_recover_home_ || !move_group_interface_)
        {
            return;
        }

        auto current_state = move_group_interface_->getCurrentState(0.2);
        if (!current_state)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to get current robot state for FK");
            return;
        }

        const auto * joint_model_group =
            current_state->getJointModelGroup(move_group_interface_->getName());
        if (!joint_model_group)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed to get joint model group: %s", move_group_interface_->getName().c_str());
            return;
        }

        std::vector<double> current_joint_values;
        current_state->copyJointGroupPositions(joint_model_group, current_joint_values);

        moveit::core::RobotState fk_state(*current_state);
        fk_state.setJointGroupPositions(joint_model_group, current_joint_values);
        fk_state.updateLinkTransforms();

        const Eigen::Isometry3d & link7_1_tf = fk_state.getGlobalLinkTransform("link7_1");
        const double link7_1_z = link7_1_tf.translation().z();

        if (link7_1_z >= z_threshold_)
        {
            return;
        }

        executing_recover_home_ = true;

        RCLCPP_WARN(get_logger(), "link7_1 z=%.4f < %.4f, trigger recover to home_gr", link7_1_z, z_threshold_);

        move_group_interface_->setNamedTarget("home_gr");
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        const bool planned =
            (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!planned)
        {
            RCLCPP_WARN(get_logger(), "Planning to home_gr failed");
            executing_recover_home_ = false;
            return;
        }

        const bool executed =
            (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (executed)
        {
            RCLCPP_INFO(get_logger(), "Recover home executed: home_gr");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Execution to home_gr failed");
        }

        executing_recover_home_ = false;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    double z_threshold_{0.045};
    bool executing_recover_home_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<RecoverHomeNode>(options);
    node->initialize_move_group();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}