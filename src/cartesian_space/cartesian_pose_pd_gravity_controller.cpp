#include "smm_controllers/cartesian_space/cartesian_pose_pd_gravity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

CartesianPosePDGravityController::CartesianPosePDGravityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CartesianPosePDGravityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    auto_declare<std::vector<double>>("kp_position", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_position", std::vector<double>{});

    auto_declare<std::vector<double>>("kp_orientation", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("x_des", std::vector<double>{});
    auto_declare<std::vector<double>>("orientation_des", std::vector<double>{});

    auto_declare<bool>("hold_initial_position", true);
    auto_declare<bool>("publish_error_state", true);
    auto_declare<bool>("publish_desired_state", true);
    auto_declare<bool>("publish_current_state", true);
    auto_declare<bool>("publish_full_debug_state", true);

    auto_declare<std::string>(
      "kinematics_data_dir",
      "/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml");

    auto_declare<std::string>(
      "dynamics_data_dir",
      "/home/nikos/ros2_ws/src/smm_class_pkgs/smm_data/synthesis/yaml");

    auto_declare<std::string>("gravity_representation", "body");
    auto_declare<std::string>("fixed_frame", "world");

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
CartesianPosePDGravityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPosePDGravityController::state_interface_configuration() const
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

controller_interface::CallbackReturn CartesianPosePDGravityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  kp_position_ = get_node()->get_parameter("kp_position").as_double_array();
  kd_position_ = get_node()->get_parameter("kd_position").as_double_array();

  kp_orientation_ = get_node()->get_parameter("kp_orientation").as_double_array();
  kd_orientation_ = get_node()->get_parameter("kd_orientation").as_double_array();

  x_des_ = get_node()->get_parameter("x_des").as_double_array();
  orientation_des_ = get_node()->get_parameter("orientation_des").as_double_array();

  hold_initial_position_ =
    get_node()->get_parameter("hold_initial_position").as_bool();

  publish_error_state_ =
    get_node()->get_parameter("publish_error_state").as_bool();

  publish_desired_state_ =
    get_node()->get_parameter("publish_desired_state").as_bool();

  publish_current_state_ =
    get_node()->get_parameter("publish_current_state").as_bool();

  publish_full_debug_state_ =
    get_node()->get_parameter("publish_full_debug_state").as_bool();

  kinematics_data_dir_ =
    get_node()->get_parameter("kinematics_data_dir").as_string();

  dynamics_data_dir_ =
    get_node()->get_parameter("dynamics_data_dir").as_string();

  gravity_representation_ =
    get_node()->get_parameter("gravity_representation").as_string();

  fixed_frame_ =
    get_node()->get_parameter("fixed_frame").as_string();

  const auto n = joint_names_.size();

  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (
    kp_position_.size() != 3 ||
    kd_position_.size() != 3 ||
    kp_orientation_.size() != 3 ||
    kd_orientation_.size() != 3)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cartesian pose gains must all have exactly 3 values. "
      "kp_position=%zu kd_position=%zu kp_orientation=%zu kd_orientation=%zu",
      kp_position_.size(),
      kd_position_.size(),
      kp_orientation_.size(),
      kd_orientation_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!x_des_.empty() && x_des_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'x_des' must be empty or have exactly 3 values. Got %zu.",
      x_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!orientation_des_.empty() && orientation_des_.size() != 4) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'orientation_des' must be empty or have exactly 4 values [x,y,z,w]. Got %zu.",
      orientation_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!expandVector3(x_des_, x_des_eig_, "x_des")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!quaternionVectorToRotation(orientation_des_, R_des_, "orientation_des")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  xdot_des_eig_.setZero();
  omega_des_.setZero();

  q_.resize(static_cast<Eigen::Index>(n));
  qdot_.resize(static_cast<Eigen::Index>(n));

  gravity_.resize(static_cast<Eigen::Index>(n));
  tau_task_.resize(static_cast<Eigen::Index>(n));
  tau_.resize(static_cast<Eigen::Index>(n));

  q_error_debug_.resize(static_cast<Eigen::Index>(n));
  qdot_error_debug_.resize(static_cast<Eigen::Index>(n));

  Jop_.resize(6, static_cast<Eigen::Index>(n));

  singular_values_.resize(std::min<size_t>(6, n));
  jacobian_column_norms_.resize(static_cast<Eigen::Index>(n));

  q_.setZero();
  qdot_.setZero();

  gravity_.setZero();
  tau_task_.setZero();
  tau_.setZero();

  q_error_debug_.setZero();
  qdot_error_debug_.setZero();

  x_.setZero();
  xdot_.setZero();
  R_.setIdentity();
  omega_.setZero();

  x_error_.setZero();
  xdot_error_.setZero();
  orientation_error_.setZero();
  omega_error_.setZero();

  force_cmd_.setZero();
  moment_cmd_.setZero();
  wrench_cmd_.setZero();

  Jop_.setZero();

  singular_values_.setZero();
  jacobian_column_norms_.setZero();
  jacobian_condition_ = 0.0;

  if (!kinematics_adapter_.initialize(kinematics_data_dir_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to initialize SmmKinematicsAdapter with kinematics_data_dir='%s'",
      kinematics_data_dir_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!dynamics_adapter_.initialize(dynamics_data_dir_, gravity_representation_)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to initialize SmmDynamicsAdapter with dynamics_data_dir='%s'",
      dynamics_data_dir_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (kinematics_adapter_.dof() != static_cast<int>(n)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "DOF mismatch. Controller joints=%zu, kinematics DOF=%d",
      n,
      kinematics_adapter_.dof());
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

  desired_cartesian_sub_ =
    get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/desired_cartesian_state",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &CartesianPosePDGravityController::desiredCartesianCallback,
        this,
        std::placeholders::_1));

  if (publish_desired_state_) {
    desired_cartesian_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/desired_cartesian_state_out",
        rclcpp::SystemDefaultsQoS());
  }

  if (publish_current_state_) {
    current_cartesian_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_cartesian_state",
        rclcpp::SystemDefaultsQoS());
  }

  if (publish_error_state_) {
    position_error_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/cartesian_error_state",
        rclcpp::SystemDefaultsQoS());

    orientation_error_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/orientation_error_state",
        rclcpp::SystemDefaultsQoS());

    error_state_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/error_state",
        rclcpp::SystemDefaultsQoS());

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

  if (publish_full_debug_state_) {
    task_force_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/task_force",
        rclcpp::SystemDefaultsQoS());

    task_moment_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/task_moment",
        rclcpp::SystemDefaultsQoS());

    task_torque_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/task_torque",
        rclcpp::SystemDefaultsQoS());

    gravity_torque_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/gravity_torque",
        rclcpp::SystemDefaultsQoS());

    commanded_torque_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/commanded_torque",
        rclcpp::SystemDefaultsQoS());

    jacobian_condition_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64>(
        "~/jacobian_condition",
        rclcpp::SystemDefaultsQoS());

    jacobian_singular_values_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/jacobian_singular_values",
        rclcpp::SystemDefaultsQoS());

    jacobian_column_norms_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/jacobian_column_norms",
        rclcpp::SystemDefaultsQoS());

    task_torque_msg_.name = joint_names_;
    gravity_torque_msg_.name = joint_names_;
    commanded_torque_msg_.name = joint_names_;
    jacobian_column_norms_msg_.name = joint_names_;

    jacobian_singular_values_msg_.name.clear();
    for (Eigen::Index i = 0; i < singular_values_.size(); ++i) {
      jacobian_singular_values_msg_.name.push_back(
        "sigma_" + std::to_string(i + 1));
    }
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured CartesianPosePDGravityController with %zu joints.", n);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Cartesian pose reference topic: ~/desired_cartesian_state");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPosePDGravityController::on_activate(
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

  if (!readStateInterfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!computeCartesianPoseKinematics()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (hold_initial_position_ || x_des_.empty()) {
    x_des_eig_ = x_;
    R_des_ = R_;

    xdot_des_eig_.setZero();
    omega_des_.setZero();

    RCLCPP_INFO(
      get_node()->get_logger(),
      "CartesianPosePDGravityController holding initial TCP pose.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPosePDGravityController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!readStateInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianPoseKinematics()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeGravityVector()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianPosePDGravityCommand()) {
    return controller_interface::return_type::ERROR;
  }

  publishDesiredCartesianState();
  publishCurrentCartesianState();
  publishCartesianErrorState();
  publishDebugState();
  publishFullDebugState();

  if (!writeCommandInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void CartesianPosePDGravityController::desiredCartesianCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!acceptDesiredCartesianState(*msg)) {
    return;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Accepted desired Cartesian TCP pose. Position: [%.4f, %.4f, %.4f]",
    x_des_eig_(0),
    x_des_eig_(1),
    x_des_eig_(2));
}

bool CartesianPosePDGravityController::acceptDesiredCartesianState(
  const geometry_msgs::msg::PoseStamped & msg)
{
  x_des_eig_(0) = msg.pose.position.x;
  x_des_eig_(1) = msg.pose.position.y;
  x_des_eig_(2) = msg.pose.position.z;

  Eigen::Quaterniond q_des(
    msg.pose.orientation.w,
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z);

  if (q_des.norm() < 1e-9) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received invalid desired orientation quaternion. Ignoring command.");
    return false;
  }

  q_des.normalize();
  R_des_ = q_des.toRotationMatrix();

  xdot_des_eig_.setZero();
  omega_des_.setZero();

  hold_initial_position_ = false;

  return true;
}

bool CartesianPosePDGravityController::readStateInterfaces()
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

bool CartesianPosePDGravityController::computeCartesianPoseKinematics()
{
  return kinematics_adapter_.computeTcpPoseKinematics(
    q_,
    qdot_,
    x_,
    R_,
    xdot_,
    omega_,
    Jop_);
}

bool CartesianPosePDGravityController::computeGravityVector()
{
  if (!dynamics_adapter_.computeGravity(q_, qdot_, gravity_)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute gravity vector.");

    return false;
  }

  return true;
}

bool CartesianPosePDGravityController::computeCartesianPosePDGravityCommand()
{
  const auto n = joint_names_.size();

  if (Jop_.rows() != 6 || Jop_.cols() != static_cast<Eigen::Index>(n)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid operational Jacobian size. Expected 6x%zu, got %ldx%ld.",
      n,
      Jop_.rows(),
      Jop_.cols());

    return false;
  }

  if (gravity_.size() != static_cast<Eigen::Index>(n)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid gravity vector size. Expected %zu, got %ld.",
      n,
      gravity_.size());

    return false;
  }

  x_error_ = x_des_eig_ - x_;
  xdot_error_ = xdot_des_eig_ - xdot_;

  orientation_error_ =
    computeOrientationError(R_des_, R_);

  omega_error_ = omega_des_ - omega_;

  for (int i = 0; i < 3; ++i) {
    force_cmd_(i) =
      kp_position_[static_cast<size_t>(i)] * x_error_(i) +
      kd_position_[static_cast<size_t>(i)] * xdot_error_(i);

    moment_cmd_(i) =
      kp_orientation_[static_cast<size_t>(i)] * orientation_error_(i) +
      kd_orientation_[static_cast<size_t>(i)] * omega_error_(i);
  }

  wrench_cmd_.segment<3>(0) = force_cmd_;
  wrench_cmd_.segment<3>(3) = moment_cmd_;

  tau_task_ = Jop_.transpose() * wrench_cmd_;
  //tau_ = tau_task_ + gravity_;
  const double task_torque_scale = 3.0;
  tau_ = gravity_ + task_torque_scale * tau_task_;

  computeJacobianConditioning();
  computeJacobianColumnNorms();

  q_error_debug_.setZero();
  qdot_error_debug_.setZero();

  return true;
}

void CartesianPosePDGravityController::computeJacobianConditioning()
{
  jacobian_condition_ = 0.0;
  singular_values_.setZero();

  if (Jop_.rows() != 6 || Jop_.cols() == 0) {
    return;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jop_);
  const Eigen::VectorXd sigma = svd.singularValues();

  const Eigen::Index count =
    std::min<Eigen::Index>(sigma.size(), singular_values_.size());

  for (Eigen::Index i = 0; i < count; ++i) {
    singular_values_(i) = sigma(i);
  }

  if (sigma.size() == 0) {
    jacobian_condition_ = 0.0;
    return;
  }

  const double sigma_max = sigma(0);
  const double sigma_min = sigma(sigma.size() - 1);

  const double eps = 1e-8;

  if (std::abs(sigma_min) < eps) {
    jacobian_condition_ = 1.0e12;
  } else {
    jacobian_condition_ = sigma_max / sigma_min;
  }
}

void CartesianPosePDGravityController::computeJacobianColumnNorms()
{
  jacobian_column_norms_.setZero();

  if (Jop_.rows() != 6) {
    return;
  }

  const Eigen::Index n_cols = Jop_.cols();

  if (jacobian_column_norms_.size() != n_cols) {
    jacobian_column_norms_.resize(n_cols);
    jacobian_column_norms_.setZero();
  }

  for (Eigen::Index j = 0; j < n_cols; ++j) {
    jacobian_column_norms_(j) = Jop_.col(j).norm();
  }
}

void CartesianPosePDGravityController::publishDesiredCartesianState()
{
  if (!publish_desired_state_ || !desired_cartesian_pub_) {
    return;
  }

  fillPoseStamped(
    desired_cartesian_msg_,
    fixed_frame_,
    get_node()->now(),
    x_des_eig_,
    R_des_);

  desired_cartesian_pub_->publish(desired_cartesian_msg_);
}

void CartesianPosePDGravityController::publishCurrentCartesianState()
{
  if (!publish_current_state_ || !current_cartesian_pub_) {
    return;
  }

  fillPoseStamped(
    current_cartesian_msg_,
    fixed_frame_,
    get_node()->now(),
    x_,
    R_);

  current_cartesian_pub_->publish(current_cartesian_msg_);
}

void CartesianPosePDGravityController::publishCartesianErrorState()
{
  if (!publish_error_state_) {
    return;
  }

  const auto now = get_node()->now();

  if (position_error_pub_) {
    position_error_msg_.header.stamp = now;
    position_error_msg_.header.frame_id = fixed_frame_;

    position_error_msg_.vector.x = x_error_(0);
    position_error_msg_.vector.y = x_error_(1);
    position_error_msg_.vector.z = x_error_(2);

    position_error_pub_->publish(position_error_msg_);
  }

  if (orientation_error_pub_) {
    orientation_error_msg_.header.stamp = now;
    orientation_error_msg_.header.frame_id = fixed_frame_;

    orientation_error_msg_.vector.x = orientation_error_(0);
    orientation_error_msg_.vector.y = orientation_error_(1);
    orientation_error_msg_.vector.z = orientation_error_(2);

    orientation_error_pub_->publish(orientation_error_msg_);
  }
}

void CartesianPosePDGravityController::publishDebugState()
{
  if (!publish_error_state_) {
    return;
  }

  if (error_state_pub_) {
    fill_joint_error_state_msg(
      error_state_msg_,
      joint_names_,
      q_error_debug_,
      qdot_error_debug_,
      tau_);

    error_state_pub_->publish(error_state_msg_);
  }

  publish_scalar_error_topics(
    q_error_pubs_,
    q_error_msgs_,
    q_error_debug_);
}

void CartesianPosePDGravityController::publishFullDebugState()
{
  if (!publish_full_debug_state_) {
    return;
  }

  const auto now = get_node()->now();

  if (task_force_pub_) {
    task_force_msg_.header.stamp = now;
    task_force_msg_.header.frame_id = fixed_frame_;
    task_force_msg_.vector.x = force_cmd_(0);
    task_force_msg_.vector.y = force_cmd_(1);
    task_force_msg_.vector.z = force_cmd_(2);
    task_force_pub_->publish(task_force_msg_);
  }

  if (task_moment_pub_) {
    task_moment_msg_.header.stamp = now;
    task_moment_msg_.header.frame_id = fixed_frame_;
    task_moment_msg_.vector.x = moment_cmd_(0);
    task_moment_msg_.vector.y = moment_cmd_(1);
    task_moment_msg_.vector.z = moment_cmd_(2);
    task_moment_pub_->publish(task_moment_msg_);
  }

  const auto n = static_cast<Eigen::Index>(joint_names_.size());
  const Eigen::VectorXd zero_n = Eigen::VectorXd::Zero(n);

  if (task_torque_pub_) {
    fill_joint_state_msg(
      task_torque_msg_,
      joint_names_,
      zero_n,
      zero_n,
      tau_task_);

    task_torque_pub_->publish(task_torque_msg_);
  }

  if (gravity_torque_pub_) {
    fill_joint_state_msg(
      gravity_torque_msg_,
      joint_names_,
      zero_n,
      zero_n,
      gravity_);

    gravity_torque_pub_->publish(gravity_torque_msg_);
  }

  if (commanded_torque_pub_) {
    fill_joint_state_msg(
      commanded_torque_msg_,
      joint_names_,
      zero_n,
      zero_n,
      tau_);

    commanded_torque_pub_->publish(commanded_torque_msg_);
  }

  if (jacobian_condition_pub_) {
    jacobian_condition_msg_.data = jacobian_condition_;
    jacobian_condition_pub_->publish(jacobian_condition_msg_);
  }

  if (jacobian_singular_values_pub_) {
    jacobian_singular_values_msg_.header.stamp = now;
    jacobian_singular_values_msg_.header.frame_id = fixed_frame_;

    jacobian_singular_values_msg_.position.resize(
      static_cast<size_t>(singular_values_.size()), 0.0);

    jacobian_singular_values_msg_.velocity.resize(
      static_cast<size_t>(singular_values_.size()), 0.0);

    jacobian_singular_values_msg_.effort.resize(
      static_cast<size_t>(singular_values_.size()), 0.0);

    for (Eigen::Index i = 0; i < singular_values_.size(); ++i) {
      jacobian_singular_values_msg_.position[static_cast<size_t>(i)] =
        singular_values_(i);
    }

    jacobian_singular_values_pub_->publish(jacobian_singular_values_msg_);
  }

  if (jacobian_column_norms_pub_) {
    fill_joint_state_msg(
      jacobian_column_norms_msg_,
      joint_names_,
      jacobian_column_norms_,
      zero_n,
      tau_task_);

    jacobian_column_norms_pub_->publish(jacobian_column_norms_msg_);
  }
}

bool CartesianPosePDGravityController::writeCommandInterfaces()
{
  return write_vector_to_command_interfaces(
    command_interfaces_,
    joint_names_,
    tau_,
    get_node()->get_logger(),
    *get_node()->get_clock(),
    "effort");
}

bool CartesianPosePDGravityController::expandVector3(
  const std::vector<double> & input,
  Eigen::Vector3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setZero();
    return true;
  }

  if (input.size() != 3) {
    std::cerr << "[CartesianPosePDGravityController] Parameter '"
              << field_name
              << "' must be empty or size 3. Got "
              << input.size()
              << "\n";
    return false;
  }

  output(0) = input[0];
  output(1) = input[1];
  output(2) = input[2];

  return true;
}

bool CartesianPosePDGravityController::quaternionVectorToRotation(
  const std::vector<double> & input,
  Eigen::Matrix3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setIdentity();
    return true;
  }

  if (input.size() != 4) {
    std::cerr << "[CartesianPosePDGravityController] Parameter '"
              << field_name
              << "' must be empty or size 4 [x,y,z,w]. Got "
              << input.size()
              << "\n";
    return false;
  }

  Eigen::Quaterniond q(
    input[3],
    input[0],
    input[1],
    input[2]);

  if (q.norm() < 1e-9) {
    std::cerr << "[CartesianPosePDGravityController] Parameter '"
              << field_name
              << "' has invalid near-zero quaternion.\n";
    return false;
  }

  q.normalize();
  output = q.toRotationMatrix();

  return true;
}

Eigen::Quaterniond CartesianPosePDGravityController::rotationToQuaternion(
  const Eigen::Matrix3d & R)
{
  Eigen::Quaterniond q(R);
  q.normalize();
  return q;
}

Eigen::Vector3d CartesianPosePDGravityController::computeOrientationError(
  const Eigen::Matrix3d & R_des,
  const Eigen::Matrix3d & R)
{
  const Eigen::Matrix3d skew_error =
    0.5 * (R_des * R.transpose() - R * R_des.transpose());

  Eigen::Vector3d e;
  e(0) = skew_error(2, 1);
  e(1) = skew_error(0, 2);
  e(2) = skew_error(1, 0);

  return e;
}

void CartesianPosePDGravityController::fillPoseStamped(
  geometry_msgs::msg::PoseStamped & msg,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const Eigen::Vector3d & position,
  const Eigen::Matrix3d & orientation)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);

  const Eigen::Quaterniond q =
    rotationToQuaternion(orientation);

  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
}

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::CartesianPosePDGravityController,
  controller_interface::ControllerInterface)