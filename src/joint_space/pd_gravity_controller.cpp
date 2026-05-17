#include "smm_controllers/joint_space/pd_gravity_controller.hpp"

#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

PDGravityController::PDGravityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn PDGravityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    auto_declare<std::vector<double>>("kp", std::vector<double>{});
    auto_declare<std::vector<double>>("kd", std::vector<double>{});

    auto_declare<std::vector<double>>("q_des", std::vector<double>{});
    auto_declare<std::vector<double>>("qdot_des", std::vector<double>{});

    auto_declare<bool>("hold_initial_position", true);
    auto_declare<bool>("publish_error_state", true);
    auto_declare<bool>("publish_desired_state", true);

    auto_declare<std::string>(
      "dynamics_data_dir",
      "/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml");

    auto_declare<std::string>("gravity_representation", "body");

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
PDGravityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
PDGravityController::state_interface_configuration() const
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

controller_interface::CallbackReturn PDGravityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();

  q_des_ = get_node()->get_parameter("q_des").as_double_array();
  qdot_des_ = get_node()->get_parameter("qdot_des").as_double_array();

  hold_initial_position_ =
    get_node()->get_parameter("hold_initial_position").as_bool();

  publish_desired_state_ =
    get_node()->get_parameter("publish_desired_state").as_bool();

  publish_error_state_ =
    get_node()->get_parameter("publish_error_state").as_bool();

  dynamics_data_dir_ =
    get_node()->get_parameter("dynamics_data_dir").as_string();

  gravity_representation_ =
    get_node()->get_parameter("gravity_representation").as_string();

  const auto n = joint_names_.size();

  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (kp_.size() != n || kd_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'kp' and 'kd' must have the same size as 'joints'. "
      "joints=%zu kp=%zu kd=%zu",
      n,
      kp_.size(),
      kd_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!q_des_.empty() && q_des_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'q_des' must be empty or have the same size as 'joints'. "
      "joints=%zu q_des=%zu",
      n,
      q_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!qdot_des_.empty() && qdot_des_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'qdot_des' must be empty or have the same size as 'joints'. "
      "joints=%zu qdot_des=%zu",
      n,
      qdot_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (qdot_des_.empty()) {
    qdot_des_.assign(n, 0.0);
  }

  q_.resize(static_cast<Eigen::Index>(n));
  qdot_.resize(static_cast<Eigen::Index>(n));

  q_des_eig_.resize(static_cast<Eigen::Index>(n));
  qdot_des_eig_.resize(static_cast<Eigen::Index>(n));

  e_.resize(static_cast<Eigen::Index>(n));
  edot_.resize(static_cast<Eigen::Index>(n));

  gravity_.resize(static_cast<Eigen::Index>(n));
  tau_.resize(static_cast<Eigen::Index>(n));

  q_.setZero();
  qdot_.setZero();

  q_des_eig_.setZero();
  qdot_des_eig_.setZero();

  e_.setZero();
  edot_.setZero();

  gravity_.setZero();
  tau_.setZero();

  for (size_t i = 0; i < n; ++i) {
    qdot_des_eig_(static_cast<Eigen::Index>(i)) = qdot_des_[i];
  }

  if (!dynamics_adapter_.initialize(dynamics_data_dir_, gravity_representation_)) {
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

  if (publish_desired_state_) {
    desired_state_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/desired_joint_state",
        rclcpp::SystemDefaultsQoS());

    desired_state_msg_.name = joint_names_;
    desired_state_msg_.position.resize(n, 0.0);
    desired_state_msg_.velocity.resize(n, 0.0);
    desired_state_msg_.effort.resize(n, 0.0);
  }  

  reference_sub_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/reference",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &PDGravityController::referenceCallback,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "PDGravityController subscribed to ~/reference.");

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured PDGravityController with %zu joints, dynamics_data_dir='%s', "
    "gravity_representation='%s'.",
    n,
    dynamics_data_dir_.c_str(),
    gravity_representation_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDGravityController::on_activate(
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
      "PDGravityController holding initial joint position.");
  }

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    q_des_eig_(idx) = q_des_[i];

    if (i < qdot_des_.size()) {
      qdot_des_eig_(idx) = qdot_des_[i];
    } else {
      qdot_des_eig_(idx) = 0.0;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PDGravityController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!readStateInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computePDGravityCommand()) {
    return controller_interface::return_type::ERROR;
  }

  publishDesiredState();
  publishDebugState();

  if (!writeCommandInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void PDGravityController::referenceCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!validateTrajectoryMessage(*msg)) {
    return;
  }

  if (msg->points.size() > 1) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "PDGravityController received %zu trajectory points. "
      "Only single-point references are currently supported. Using first point.",
      msg->points.size());
  }

  acceptSinglePointCommand(*msg);
}

bool PDGravityController::validateTrajectoryMessage(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  const auto n = joint_names_.size();

  if (trajectory.points.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received empty JointTrajectory reference. Ignoring command.");
    return false;
  }

  const auto & point = trajectory.points.front();

  if (point.positions.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Reference position size mismatch. Expected %zu, got %zu.",
      n,
      point.positions.size());
    return false;
  }

  if (!point.velocities.empty() && point.velocities.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Reference velocity size mismatch. Expected %zu, got %zu.",
      n,
      point.velocities.size());
    return false;
  }

  return true;
}

bool PDGravityController::acceptSinglePointCommand(
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
  }

  hold_initial_position_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "PDGravityController accepted single-point reference with %zu joints.",
    n);

  return true;
}

bool PDGravityController::readStateInterfaces()
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

bool PDGravityController::computePDGravityCommand()
{
  const auto n = joint_names_.size();

  if (!dynamics_adapter_.computeGravity(q_, qdot_, gravity_)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute gravity vector.");

    return false;
  }

  e_ = q_des_eig_ - q_;
  edot_ = qdot_des_eig_ - qdot_;

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    const double tau_pd =
      kp_[i] * e_(idx) +
      kd_[i] * edot_(idx);

    tau_(idx) = tau_pd + gravity_(idx);
  }

  return true;
}

void PDGravityController::publishDesiredState()
{
  if (!publish_desired_state_ || !desired_state_pub_) {
    return;
  }

  fill_joint_state_msg(
    desired_state_msg_,
    joint_names_,
    q_des_eig_,
    qdot_des_eig_,
    tau_);

  desired_state_pub_->publish(desired_state_msg_);
}

void PDGravityController::publishDebugState()
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

bool PDGravityController::writeCommandInterfaces()
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
  smm_controllers::PDGravityController,
  controller_interface::ControllerInterface)