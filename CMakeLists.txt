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
find_package(smm_screws REQUIRED)

add_library(smm_controller_core SHARED
  src/core/smm_controller_utils.cpp
  src/core/smm_dynamics_adapter.cpp
)

target_include_directories(smm_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(smm_controller_core
  PUBLIC
    rclcpp::rclcpp
    Eigen3::Eigen
    yaml-cpp
    smm_screws::RobotContextLibNdof
    smm_screws::ScrewsDynamicsNdof
    smm_screws::ScrewsKinematicsNdof
    smm_screws::robot_yaml_loader_ndof
)

add_library(smm_joint_space_controllers SHARED
  src/joint_space/joint_pd_effort_controller.cpp
  src/joint_space/pd_gravity_controller.cpp
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
)

pluginlib_export_plugin_description_file(
  controller_interface
  smm_joint_space_controllers.xml
)

install(
  DIRECTORY include/
  DESTINATION include
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
  FILES smm_joint_space_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_smm_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  smm_screws
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()