#!/bin/bash
# AutoGantryEngineer - ROS2 Jazzy 环境设置脚本
# 用法: source setup_jazzy.sh

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "ROS2 Jazzy 环境已设置"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  RMW: $RMW_IMPLEMENTATION"

# 如果存在本地 install，也 source 本地 setup
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
  echo "  Workspace: $(pwd)"
fi
