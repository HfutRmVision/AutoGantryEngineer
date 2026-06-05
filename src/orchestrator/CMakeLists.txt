cmake_minimum_required(VERSION 3.8)
project(orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rm_serial_driver REQUIRED)
find_package(controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find BehaviorTree.CPP
find_package(behaviortree_cpp REQUIRED)

add_executable(serial_test_node src/serial_test_node.cpp)
ament_target_dependencies(serial_test_node 
  rclcpp 
  moveit_msgs
  rm_serial_driver)

# Add BehaviorTree-based orchestrator
add_executable(bt_orchestrator_node src/bt_orchestrator_node.cpp)
ament_target_dependencies(bt_orchestrator_node
  ament_index_cpp
  rclcpp
  moveit_msgs
  controller
  std_msgs
  moveit_task_constructor_core
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  behaviortree_cpp)
target_include_directories(bt_orchestrator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS serial_test_node bt_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
