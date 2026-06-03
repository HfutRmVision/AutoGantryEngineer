// Copyright (c) 2022-2026, b»robotized group (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include <vector>

#include "gantry_robot_hardware_interface/gantry_robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/publisher.hpp"

namespace gantry_robot_hardware_interface
{

hardware_interface::CallbackReturn GantryRobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 验证关节数量
  if (info_.joints.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(logger_, "Expected %zu joints, but got %zu", NUM_JOINTS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // 读取串口参数
  serial_port_ = info_.hardware_parameters.count("serial_port") ? 
                 info_.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
  enable_data_print_ = info_.hardware_parameters.count("enable_data_print") ? 
                       (info_.hardware_parameters.at("enable_data_print") == "true") : false;

  RCLCPP_INFO(logger_, "Serial port: %s", serial_port_.c_str());

  // 初始化状态和命令向量
  hw_commands_position_.resize(NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.resize(NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(NUM_JOINTS, 0.0);

  RCLCPP_INFO(logger_, "GantryRobotHardwareInterface initialized successfully");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GantryRobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring hardware interface...");

  // 创建 ROS 节点用于服务调用
  node_ = rclcpp::Node::make_shared("gantry_robot_hw_node");
  switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");

  // 使用已有的 EngineerProtocol 类
  try
  {
    protocol_ = std::make_unique<fyt::serial_driver::protocol::EngineerProtocol>(
      serial_port_, enable_data_print_);
    RCLCPP_INFO(logger_, "EngineerProtocol created successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "Failed to create EngineerProtocol: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // 使用默认初始位置（不阻塞等待硬件数据）
  // 硬件数据将在 read() 循环中获取
  RCLCPP_INFO(logger_, "Using default initial positions (hardware data will be read in control loop)");
  hw_states_position_[0] = -0.3;    // joint_1_2
  hw_states_position_[1] = 3.1416;  // joint_2_3
  hw_states_position_[2] = 3.1416;  // joint_3_4
  hw_states_position_[3] = 0.0;     // joint_4_5
  hw_states_position_[4] = 0.0;     // joint_5_6
  hw_states_position_[5] = 0.0;     // joint_6_7
  hw_commands_position_ = hw_states_position_;

  RCLCPP_INFO(logger_, "Hardware interface configured successfully");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GantryRobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GantryRobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GantryRobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // 设置初始命令为当前位置
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    if (std::isnan(hw_commands_position_[i]))
    {
      hw_commands_position_[i] = hw_states_position_[i];
    }
  }

  communication_active_ = true;
  jtc_active_ = true;  // 初始状态假设 JTC 是激活的

  RCLCPP_INFO(logger_, "Hardware interface activated successfully");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GantryRobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  
  communication_active_ = false;
  protocol_.reset();

  RCLCPP_INFO(logger_, "Hardware interface deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GantryRobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!communication_active_ || !protocol_)
  {
    return hardware_interface::return_type::OK;
  }

  // 使用 EngineerProtocol::receive() 读取数据
  float data[16];
  if (protocol_->receive(data))
  {
    // data[0] = mode
    // data[1-6] = positions (已经转换为ROS坐标系)
    // data[7-12] = velocities (已经转换为ROS坐标系)
    
    // 更新模式标志：mode == 3 时允许 write
    is_ros_control_mode_ = (static_cast<int>(data[0]) == 3);
    
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      hw_states_position_[i] = static_cast<double>(data[i + 1]);
      hw_states_velocity_[i] = static_cast<double>(data[i + 7]);
    }

    if(!is_ros_control_mode_){
      // mode != 3 时，关闭 JTC，防止manual与ros_control冲突
      if (jtc_active_) {
        deactivate_jtc();
      }
      update_current_state();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GantryRobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/){
  if (!communication_active_ || !protocol_)
  {
    return hardware_interface::return_type::OK;
  }

  // 检查命令是否有效
  bool has_valid_command = false;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    if (!std::isnan(hw_commands_position_[i]))
    {
      has_valid_command = true;
      break;
    }
  }

  // 仅在 ros_control 模式 (mode == 3) 时执行 write
  if (!is_ros_control_mode_)
  {
    return hardware_interface::return_type::OK;
  }

  // 激活JTC
  if (!jtc_active_) {
    activate_jtc();
  }

  if (has_valid_command)
  {
    moveit_msgs::msg::DisplayTrajectory trajectory_msg;
    trajectory_msg.trajectory.resize(1);
    trajectory_msg.trajectory[0].joint_trajectory.points.resize(1);
    trajectory_msg.trajectory[0].joint_trajectory.points[0].positions.resize(NUM_JOINTS);
    
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      trajectory_msg.trajectory[0].joint_trajectory.points[0].positions[i] = hw_commands_position_[i];
    }
    
    protocol_->send(trajectory_msg, 0);
  }

  return hardware_interface::return_type::OK;
}

void GantryRobotHardwareInterface::update_current_state()
{
  moveit_msgs::msg::DisplayTrajectory trajectory_msg;
  trajectory_msg.trajectory.resize(1);
  trajectory_msg.trajectory[0].joint_trajectory.points.resize(1);
  trajectory_msg.trajectory[0].joint_trajectory.points[0].positions.resize(NUM_JOINTS);
  
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    trajectory_msg.trajectory[0].joint_trajectory.points[0].positions[i] = hw_states_position_[i];
    hw_commands_position_[i] = hw_states_position_[i];
  }      

  protocol_->send(trajectory_msg, 0);
}

void GantryRobotHardwareInterface::deactivate_jtc()
{
  if (!switch_controller_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(logger_, "Switch controller service not available");
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = {"gantry_robot_controller"};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

  auto future = switch_controller_client_->async_send_request(request);
  
  // 使用 spin_some 来处理响应，避免阻塞
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(500)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->ok) {
      jtc_active_ = false;
      RCLCPP_INFO(logger_, "JTC deactivated successfully");
    } else {
      RCLCPP_WARN(logger_, "Failed to deactivate JTC");
    }
  } else {
    RCLCPP_WARN(logger_, "Timeout waiting for JTC deactivation");
  }
}

void GantryRobotHardwareInterface::activate_jtc()
{
  if (!switch_controller_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(logger_, "Switch controller service not available");
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->activate_controllers = {"gantry_robot_controller"};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

  auto future = switch_controller_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(500)) == 
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = future.get();
    if (result->ok) {
      jtc_active_ = true;
      RCLCPP_INFO(logger_, "JTC activated successfully");
    } else {
      RCLCPP_WARN(logger_, "Failed to activate JTC");
    }
  } else {
    RCLCPP_WARN(logger_, "Timeout waiting for JTC activation");
  }
}

}  // namespace gantry_robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gantry_robot_hardware_interface::GantryRobotHardwareInterface, hardware_interface::SystemInterface)
