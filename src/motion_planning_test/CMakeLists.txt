cmake_minimum_required(VERSION 3.8)
project(motion_planning_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set hints for Bullet library search
set(BULLET_ROOT "/usr")
set(ENV{BULLET_ROOT} "/usr")

# Manually set Bullet variables to bypass FindBullet issues
set(BULLET_INCLUDE_DIR "/usr/include/bullet" CACHE PATH "Bullet include directory")
set(BULLET_DYNAMICS_LIBRARY "/usr/lib/x86_64-linux-gnu/libBulletDynamics.so" CACHE FILEPATH "Bullet Dynamics library")
set(BULLET_COLLISION_LIBRARY "/usr/lib/x86_64-linux-gnu/libBulletCollision.so" CACHE FILEPATH "Bullet Collision library")
set(BULLET_MATH_LIBRARY "/usr/lib/x86_64-linux-gnu/libLinearMath.so" CACHE FILEPATH "Bullet Math library")
set(BULLET_SOFTBODY_LIBRARY "/usr/lib/x86_64-linux-gnu/libBulletSoftBody.so" CACHE FILEPATH "Bullet SoftBody library")
set(BULLET_FOUND TRUE CACHE BOOL "Bullet found")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable for end effector alignment test
add_executable(ee_alignment_test 
  src/ee_alignment_test.cpp
)

ament_target_dependencies(ee_alignment_test
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_task_constructor_core
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Add executable for stand random pose test
add_executable(stand_random_pose 
  src/stand_random_pose.cpp
)

ament_target_dependencies(stand_random_pose
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Add executable for target scene publisher
add_executable(target_scene_publisher
  src/target_scene_publisher.cpp
)

ament_target_dependencies(target_scene_publisher
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# Install executable
install(TARGETS
  ee_alignment_test
  stand_random_pose
  target_scene_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
