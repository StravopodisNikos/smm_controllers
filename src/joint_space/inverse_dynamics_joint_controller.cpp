#include "smm_controllers/joint_space/inverse_dynamics_joint_controller.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"
#include "smm_controllers/core/trajectory_operation_utils.hpp"

/*
This is the class that implements the inverse dynamics joint controller

smm_controller_utils
  generic ROS-control helper operations:
    make_interface_names()
    write_vector_to_command_interfaces()
    fill_joint_error_state_msg()
    publish_scalar_error_topics()

inverse_dynamics_joint_controller
  controller-specific logic:
    readStateInterfaces()
    computeInverseDynamicsCommand()
    processReferenceCommandIfAvailable()
    sampleActiveTrajectory()

*/
namespace smm_controllers
{

InverseDynamicsJointController::InverseDynamicsJointController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn InverseDynamicsJointController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    auto_declare<std::vector<double>>("kp", std::vector<double>{});
    auto_declare<std::vector<double>>("kd", std::vector<double>{});

    auto_declare<std::vector<double>>("q_des", std::vector<double>{});
    auto_declare<std::vector<double>>("qdot_des", std::vector<double>{});
    auto_declare<std::vector<double>>("qddot_des", std::vector<double>{});

    auto_declare<bool>("hold_initial_position", true);

    auto_declare<std::string>(
      "dynamics_data_dir",
      "/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml");

    auto_declare<std::string>("dynamics_representation", "body");
    auto_declare<bool>("publish_error_state", true);

    auto_declare<std::string>("trajectory_interpolation_mode", "sync_cubic");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception during on_init: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
InverseDynamicsJointController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
InverseDynamicsJointController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  auto position_names =
    make_interface_names(joint_names_, hardware_interface::HW_IF_POSITION);
  auto velocity_names =
    make_interface_names(joint_names_, hardware_interface::HW_IF_VELOCITY);

  config.names.reserve(position_names.size() + velocity_names.size());
  config.names.insert(config.names.end(), position_names.begin(), position_names.end());
  config.names.insert(config.names.end(), velocity_names.begin(), velocity_names.end());

  return config;
}

controller_interface::CallbackReturn InverseDynamicsJointController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();

  q_des_ = get_node()->get_parameter("q_des").as_double_array();
  qdot_des_ = get_node()->get_parameter("qdot_des").as_double_array();
  qddot_des_ = get_node()->get_parameter("qddot_des").as_double_array();

  hold_initial_position_ =
    get_node()->get_parameter("hold_initial_position").as_bool();

  publish_error_state_ =
    get_node()->get_parameter("publish_error_state").as_bool();

  dynamics_data_dir_ =
    get_node()->get_parameter("dynamics_data_dir").as_string();

  dynamics_representation_ =
    get_node()->get_parameter("dynamics_representation").as_string();

  trajectory_interpolation_mode_str_ =
    get_node()->get_parameter("trajectory_interpolation_mode").as_string();

  try {
    trajectory_interpolation_mode_ =
      TrajectoryOperationUtils::parseInterpolationMode(
        trajectory_interpolation_mode_str_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid trajectory_interpolation_mode='%s': %s",
      trajectory_interpolation_mode_str_.c_str(),
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto n = joint_names_.size();

  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (kp_.size() != n || kd_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'kp' and 'kd' must match joints size. joints=%zu kp=%zu kd=%zu",
      n,
      kp_.size(),
      kd_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!q_des_.empty() && q_des_.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "q_des must be empty or size %zu.", n);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!qdot_des_.empty() && qdot_des_.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "qdot_des must be empty or size %zu.", n);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!qddot_des_.empty() && qddot_des_.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "qddot_des must be empty or size %zu.", n);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (qdot_des_.empty()) {
    qdot_des_.assign(n, 0.0);
  }

  if (qddot_des_.empty()) {
    qddot_des_.assign(n, 0.0);
  }

  q_.resize(static_cast<Eigen::Index>(n));
  qdot_.resize(static_cast<Eigen::Index>(n));
  q_des_eig_.resize(static_cast<Eigen::Index>(n));
  qdot_des_eig_.resize(static_cast<Eigen::Index>(n));
  qddot_des_eig_.resize(static_cast<Eigen::Index>(n));

  e_.resize(static_cast<Eigen::Index>(n));
  edot_.resize(static_cast<Eigen::Index>(n));
  v_.resize(static_cast<Eigen::Index>(n));
  gravity_.resize(static_cast<Eigen::Index>(n));
  coriolis_vector_.resize(static_cast<Eigen::Index>(n));
  tau_.resize(static_cast<Eigen::Index>(n));

  mass_matrix_.resize(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n));
  coriolis_matrix_.resize(static_cast<Eigen::Index>(n), static_cast<Eigen::Index>(n));

  q_.setZero();
  qdot_.setZero();
  q_des_eig_.setZero();
  qdot_des_eig_.setZero();
  qddot_des_eig_.setZero();
  e_.setZero();
  edot_.setZero();
  v_.setZero();
  gravity_.setZero();
  coriolis_vector_.setZero();
  tau_.setZero();
  mass_matrix_.setZero();
  coriolis_matrix_.setZero();

  for (size_t i = 0; i < n; ++i) {
    qdot_des_eig_(static_cast<Eigen::Index>(i)) = qdot_des_[i];
    qddot_des_eig_(static_cast<Eigen::Index>(i)) = qddot_des_[i];
  }

  if (publish_error_state_) {
    error_state_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/error_state",
        rclcpp::SystemDefaultsQoS());

    error_state_msg_.name = joint_names_;
    error_state_msg_.position.resize(n, 0.0);
    error_state_msg_.velocity.resize(n, 0.0);
    error_state_msg_.effort.resize(n, 0.0);

    q_error_pubs_.clear();
    q_error_msgs_.clear();

    q_error_pubs_.reserve(n);
    q_error_msgs_.resize(n);

    for (size_t i = 0; i < n; ++i) {
      const std::string topic_name =
        "~/q_error_" + std::to_string(i);

      q_error_pubs_.push_back(
        get_node()->create_publisher<std_msgs::msg::Float64>(
          topic_name,
          rclcpp::SystemDefaultsQoS()));
    }
  }

  reference_sub_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/smm_joint_controller/reference",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &InverseDynamicsJointController::referenceCallback,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "InverseDynamicsJointController subscribed to /smm_joint_controller/reference");

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Trajectory interpolation mode: %s",
    trajectory_interpolation_mode_str_.c_str());

  if (!dynamics_adapter_.initialize(dynamics_data_dir_, dynamics_representation_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to initialize SmmDynamicsAdapter with dynamics_data_dir='%s'",
      dynamics_data_dir_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (dynamics_adapter_.dof() != static_cast<int>(n)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "DOF mismatch. Controller joints=%zu, dynamics DOF=%d",
      n,
      dynamics_adapter_.dof());
    return controller_interface::CallbackReturn::ERROR;
  }

  reference_mode_ = ReferenceMode::HOLD;
  trajectory_running_ = false;
  trajectory_elapsed_sec_ = 0.0;
  active_command_.reset();
  last_processed_command_.reset();

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured InverseDynamicsJointController with %zu joints.", n);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InverseDynamicsJointController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto n = joint_names_.size();

  if (command_interfaces_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu effort command interfaces, got %zu.",
      n,
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 2 * n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu.",
      2 * n,
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (hold_initial_position_ || q_des_.empty()) {
    q_des_.resize(n);

    for (size_t i = 0; i < n; ++i) {
      const auto q_opt = state_interfaces_[i].get_optional();

      if (!q_opt.has_value()) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to read initial position for joint '%s'.",
          joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      q_des_[i] = q_opt.value();
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "InverseDynamicsJointController holding initial position.");
  }

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    q_des_eig_(idx) = q_des_[i];

    if (i < qdot_des_.size()) {
      qdot_des_eig_(idx) = qdot_des_[i];
    } else {
      qdot_des_eig_(idx) = 0.0;
    }

    if (i < qddot_des_.size()) {
      qddot_des_eig_(idx) = qddot_des_[i];
    } else {
      qddot_des_eig_(idx) = 0.0;
    }
  }

  reference_mode_ = ReferenceMode::HOLD;
  trajectory_running_ = false;
  trajectory_elapsed_sec_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type InverseDynamicsJointController::update(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  processReferenceCommandIfAvailable(period);

  if (!readStateInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeInverseDynamicsCommand()) {
    return controller_interface::return_type::ERROR;
  }

  publishDebugState();

  if (!writeCommandInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void InverseDynamicsJointController::processReferenceCommandIfAvailable(
  const rclcpp::Duration & period)
{
  auto command_ptr = reference_buffer_.readFromRT();

  if (!command_ptr || !(*command_ptr)) {
    if (reference_mode_ == ReferenceMode::TRAJECTORY && trajectory_running_) {
      trajectory_elapsed_sec_ += period.seconds();
      sampleActiveTrajectory(trajectory_elapsed_sec_);
    }
    return;
  }

  const auto command = *command_ptr;

  if (command == last_processed_command_) {
    if (reference_mode_ == ReferenceMode::TRAJECTORY && trajectory_running_) {
      trajectory_elapsed_sec_ += period.seconds();
      sampleActiveTrajectory(trajectory_elapsed_sec_);
    }
    return;
  }

  last_processed_command_ = command;
  active_command_ = command;

  if (command->trajectory.points.size() == 1) {
    acceptSinglePointCommand(command->trajectory);
  } else {
    startTrajectoryCommand(command->trajectory);
  }
}

void InverseDynamicsJointController::referenceCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  /*
   * Read interpolation mode in the non-real-time callback.
   * This allows send_joint_trajectory.py to set the parameter immediately
   * before publishing the trajectory.
   */
  const auto mode_str =
    get_node()->get_parameter("trajectory_interpolation_mode").as_string();

  try {
    trajectory_interpolation_mode_ =
      TrajectoryOperationUtils::parseInterpolationMode(mode_str);
    trajectory_interpolation_mode_str_ = mode_str;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Invalid trajectory_interpolation_mode='%s': %s",
      mode_str.c_str(),
      e.what());
    return;
  }

  if (!validateTrajectoryMessage(*msg)) {
    return;
  }

  auto command = std::make_shared<JointReferenceCommand>();
  command->trajectory = *msg;
  command->receive_time = get_node()->now();

  reference_buffer_.writeFromNonRT(command);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Received JointTrajectory reference with %zu point(s), interpolation='%s'.",
    msg->points.size(),
    trajectory_interpolation_mode_str_.c_str());
}

bool InverseDynamicsJointController::acceptSinglePointCommand(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  const auto n = joint_names_.size();
  const auto & point = trajectory.points.front();

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    q_des_[i] = point.positions[i];
    q_des_eig_(idx) = point.positions[i];

    if (!point.velocities.empty()) {
      qdot_des_[i] = point.velocities[i];
      qdot_des_eig_(idx) = point.velocities[i];
    } else {
      qdot_des_[i] = 0.0;
      qdot_des_eig_(idx) = 0.0;
    }

    if (!point.accelerations.empty()) {
      qddot_des_[i] = point.accelerations[i];
      qddot_des_eig_(idx) = point.accelerations[i];
    } else {
      qddot_des_[i] = 0.0;
      qddot_des_eig_(idx) = 0.0;
    }
  }

  reference_mode_ = ReferenceMode::SINGLE_POINT;
  trajectory_running_ = false;
  trajectory_elapsed_sec_ = 0.0;
  hold_initial_position_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Accepted single-point joint reference with %zu joints.",
    n);

  return true;
}

bool InverseDynamicsJointController::startTrajectoryCommand(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (!active_command_) {
    return false;
  }

  trajectory_elapsed_sec_ = 0.0;
  trajectory_running_ = true;
  reference_mode_ = ReferenceMode::TRAJECTORY;
  hold_initial_position_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Started joint trajectory with %zu points using interpolation='%s'.",
    trajectory.points.size(),
    trajectory_interpolation_mode_str_.c_str());

  return sampleActiveTrajectory(trajectory_elapsed_sec_);
}

bool InverseDynamicsJointController::validateTrajectoryMessage(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  const auto n = joint_names_.size();

  if (trajectory.points.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received empty JointTrajectory reference. Ignoring command.");
    return false;
  }

  for (size_t p = 0; p < trajectory.points.size(); ++p) {
    const auto & point = trajectory.points[p];

    if (point.positions.size() != n) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Trajectory point %zu position size mismatch. Expected %zu, got %zu.",
        p,
        n,
        point.positions.size());
      return false;
    }

    if (!point.velocities.empty() && point.velocities.size() != n) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Trajectory point %zu velocity size mismatch. Expected %zu, got %zu.",
        p,
        n,
        point.velocities.size());
      return false;
    }

    if (!point.accelerations.empty() && point.accelerations.size() != n) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Trajectory point %zu acceleration size mismatch. Expected %zu, got %zu.",
        p,
        n,
        point.accelerations.size());
      return false;
    }
  }

  if (trajectory.points.size() > 1) {
    double previous_time =
      TrajectoryOperationUtils::pointTimeSec(trajectory.points.front());

    for (size_t p = 1; p < trajectory.points.size(); ++p) {
      const double current_time =
        TrajectoryOperationUtils::pointTimeSec(trajectory.points[p]);

      if (current_time <= previous_time) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Trajectory time_from_start must be strictly increasing. "
          "Point %zu has %.6f s after %.6f s.",
          p,
          current_time,
          previous_time);
        return false;
      }

      previous_time = current_time;
    }
  }

  return true;
}

bool InverseDynamicsJointController::sampleActiveTrajectory(double elapsed)
{
  if (!active_command_) {
    return false;
  }

  const auto & trajectory = active_command_->trajectory;
  const auto & points = trajectory.points;
  const auto n_points = points.size();

  if (n_points == 0) {
    return false;
  }

  if (n_points == 1) {
    return acceptSinglePointCommand(trajectory);
  }

  const double final_time =
    TrajectoryOperationUtils::pointTimeSec(points.back());

  if (elapsed >= final_time) {
    const auto & final_point = points.back();

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto idx = static_cast<Eigen::Index>(i);

      q_des_[i] = final_point.positions[i];
      q_des_eig_(idx) = final_point.positions[i];

      qdot_des_[i] = 0.0;
      qdot_des_eig_(idx) = 0.0;

      qddot_des_[i] = 0.0;
      qddot_des_eig_(idx) = 0.0;
    }

    trajectory_running_ = false;
    trajectory_elapsed_sec_ = final_time;
    reference_mode_ = ReferenceMode::SINGLE_POINT;

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Joint trajectory completed. Holding final point.");

    return true;
  }

  size_t segment_index = 0;
  bool segment_found = false;

  for (size_t i = 0; i < n_points - 1; ++i) {
    const double t0 =
      TrajectoryOperationUtils::pointTimeSec(points[i]);
    const double t1 =
      TrajectoryOperationUtils::pointTimeSec(points[i + 1]);

    if (elapsed >= t0 && elapsed < t1) {
      segment_index = i;
      segment_found = true;
      break;
    }
  }

  if (!segment_found) {
    if (elapsed < TrajectoryOperationUtils::pointTimeSec(points.front())) {
      segment_index = 0;
    } else {
      segment_index = n_points - 2;
    }
  }

  Eigen::VectorXd q_sample;
  Eigen::VectorXd qdot_sample;
  Eigen::VectorXd qddot_sample;

  if (!TrajectoryOperationUtils::sampleSegment(
      trajectory_interpolation_mode_,
      points[segment_index],
      points[segment_index + 1],
      elapsed,
      joint_names_.size(),
      q_sample,
      qdot_sample,
      qddot_sample))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to sample active trajectory segment.");
    return false;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    q_des_[i] = q_sample(idx);
    qdot_des_[i] = qdot_sample(idx);
    qddot_des_[i] = qddot_sample(idx);

    q_des_eig_(idx) = q_sample(idx);
    qdot_des_eig_(idx) = qdot_sample(idx);
    qddot_des_eig_(idx) = qddot_sample(idx);
  }

  return true;
}

bool InverseDynamicsJointController::readStateInterfaces()
{
  const auto n = joint_names_.size();

  for (size_t i = 0; i < n; ++i) {
    const auto q_opt = state_interfaces_[i].get_optional();
    const auto qdot_opt = state_interfaces_[n + i].get_optional();

    if (!q_opt.has_value() || !qdot_opt.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Failed to read state interfaces for joint '%s'.",
        joint_names_[i].c_str());

      return false;
    }

    q_(static_cast<Eigen::Index>(i)) = q_opt.value();
    qdot_(static_cast<Eigen::Index>(i)) = qdot_opt.value();
  }

  return true;
}

bool InverseDynamicsJointController::computeInverseDynamicsCommand()
{
  const auto n = joint_names_.size();

  if (!dynamics_adapter_.computeJointDynamics(
      q_,
      qdot_,
      mass_matrix_,
      coriolis_matrix_,
      gravity_))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute joint dynamics.");

    return false;
  }

  e_ = q_des_eig_ - q_;
  edot_ = qdot_des_eig_ - qdot_;

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    v_(idx) =
      qddot_des_eig_(idx) +
      kd_[i] * edot_(idx) +
      kp_[i] * e_(idx);
  }

  coriolis_vector_ = coriolis_matrix_ * qdot_;

  tau_ = mass_matrix_ * v_ + coriolis_vector_ + gravity_;

  return true;
}

void InverseDynamicsJointController::publishDebugState()
{
  if (!publish_error_state_) {
    return;
  }

  if (error_state_pub_) {
    fill_joint_error_state_msg(
      error_state_msg_,
      joint_names_,
      e_,
      edot_,
      tau_);

    error_state_pub_->publish(error_state_msg_);
  }

  publish_scalar_error_topics(
    q_error_pubs_,
    q_error_msgs_,
    e_);
}

bool InverseDynamicsJointController::writeCommandInterfaces()
{
  return write_vector_to_command_interfaces(
    command_interfaces_,
    joint_names_,
    tau_,
    get_node()->get_logger(),
    *get_node()->get_clock(),
    "effort");
}

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::InverseDynamicsJointController,
  controller_interface::ControllerInterface)