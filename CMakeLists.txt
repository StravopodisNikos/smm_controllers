cmake_minimum_required(VERSION 3.20)
project(smm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(realtime_tools REQUIRED)
find_package(smm_screws REQUIRED)

# =============================================================================
# Core controller utilities library
# =============================================================================
add_library(smm_controller_core SHARED
  src/core/smm_controller_utils.cpp
  src/core/smm_dynamics_adapter.cpp
  src/core/trajectory_operation_utils.cpp
)

target_include_directories(smm_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(smm_controller_core
  PUBLIC
    rclcpp::rclcpp
    hardware_interface::hardware_interface

    Eigen3::Eigen
    yaml-cpp

    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${trajectory_msgs_TARGETS}

    smm_screws::RobotContextLibNdof
    smm_screws::ScrewsDynamicsNdof
    smm_screws::ScrewsKinematicsNdof
    smm_screws::robot_yaml_loader_ndof
)

# =============================================================================
# Joint-space controller plugin library
# =============================================================================
add_library(smm_joint_space_controllers SHARED
  src/joint_space/joint_pd_effort_controller.cpp
  src/joint_space/pd_gravity_controller.cpp
  src/joint_space/inverse_dynamics_joint_controller.cpp
)

target_include_directories(smm_joint_space_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(smm_joint_space_controllers
  PUBLIC
    smm_controller_core

    controller_interface::controller_interface
    hardware_interface::hardware_interface
    pluginlib::pluginlib

    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle

    realtime_tools::realtime_tools

    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${trajectory_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  controller_interface
  smm_joint_space_controllers.xml
)

# =============================================================================
# Install
# =============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS
    smm_controller_core
    smm_joint_space_controllers
  EXPORT export_smm_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/send_joint_goal.py
    scripts/send_joint_trajectory.py
    scripts/plot_joint_pos_error.py
    scripts/send_joint_goal_error_fdbk.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES smm_joint_space_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

# =============================================================================
# Export
# =============================================================================
ament_export_targets(export_smm_controllers HAS_LIBRARY_TARGET)

ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib

  rclcpp
  rclcpp_lifecycle

  std_msgs
  sensor_msgs
  trajectory_msgs

  realtime_tools
  smm_screws

  Eigen3
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()