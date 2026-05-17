#include "smm_controllers/cartesian_space/cartesian_pd_gravity_controller.hpp"

#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

CartesianPDGravityController::CartesianPDGravityController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CartesianPDGravityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    auto_declare<std::vector<double>>("kp_cartesian", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_cartesian", std::vector<double>{});

    auto_declare<std::vector<double>>("x_des", std::vector<double>{});
    auto_declare<std::vector<double>>("xdot_des", std::vector<double>{});

    auto_declare<bool>("hold_initial_position", true);
    auto_declare<bool>("publish_error_state", true);
    auto_declare<bool>("publish_desired_state", true);
    auto_declare<bool>("publish_current_state", true);

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
CartesianPDGravityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPDGravityController::state_interface_configuration() const
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

controller_interface::CallbackReturn CartesianPDGravityController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  kp_cartesian_ = get_node()->get_parameter("kp_cartesian").as_double_array();
  kd_cartesian_ = get_node()->get_parameter("kd_cartesian").as_double_array();

  x_des_ = get_node()->get_parameter("x_des").as_double_array();
  xdot_des_ = get_node()->get_parameter("xdot_des").as_double_array();

  hold_initial_position_ =
    get_node()->get_parameter("hold_initial_position").as_bool();

  publish_error_state_ =
    get_node()->get_parameter("publish_error_state").as_bool();

  publish_desired_state_ =
    get_node()->get_parameter("publish_desired_state").as_bool();

  publish_current_state_ =
    get_node()->get_parameter("publish_current_state").as_bool();

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

  if (kp_cartesian_.size() != 3 || kd_cartesian_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'kp_cartesian' and 'kd_cartesian' must have exactly 3 values. "
      "kp_cartesian=%zu kd_cartesian=%zu",
      kp_cartesian_.size(),
      kd_cartesian_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!x_des_.empty() && x_des_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'x_des' must be empty or have exactly 3 values. Got %zu.",
      x_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!xdot_des_.empty() && xdot_des_.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'xdot_des' must be empty or have exactly 3 values. Got %zu.",
      xdot_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!expandCartesianVector(xdot_des_, xdot_des_eig_, "xdot_des")) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!x_des_.empty()) {
    if (!expandCartesianVector(x_des_, x_des_eig_, "x_des")) {
      return controller_interface::CallbackReturn::ERROR;
    }
  } else {
    x_des_eig_.setZero();
  }

  q_.resize(static_cast<Eigen::Index>(n));
  qdot_.resize(static_cast<Eigen::Index>(n));

  gravity_.resize(static_cast<Eigen::Index>(n));
  tau_.resize(static_cast<Eigen::Index>(n));

  q_error_debug_.resize(static_cast<Eigen::Index>(n));
  qdot_error_debug_.resize(static_cast<Eigen::Index>(n));

  Jv_.resize(3, static_cast<Eigen::Index>(n));

  q_.setZero();
  qdot_.setZero();
  gravity_.setZero();
  tau_.setZero();
  q_error_debug_.setZero();
  qdot_error_debug_.setZero();

  x_.setZero();
  xdot_.setZero();
  x_error_.setZero();
  xdot_error_.setZero();
  f_cmd_.setZero();
  Jv_.setZero();

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
        &CartesianPDGravityController::desiredCartesianCallback,
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
    cartesian_error_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/cartesian_error_state",
        rclcpp::SystemDefaultsQoS());

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

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured CartesianPDGravityController with %zu joints.", n);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Cartesian reference topic: ~/desired_cartesian_state");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPDGravityController::on_activate(
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

  if (!computeCartesianKinematics()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (hold_initial_position_ || x_des_.empty()) {
    x_des_eig_ = x_;
    xdot_des_eig_.setZero();

    RCLCPP_INFO(
      get_node()->get_logger(),
      "CartesianPDGravityController holding initial TCP position: [%.4f, %.4f, %.4f]",
      x_des_eig_(0),
      x_des_eig_(1),
      x_des_eig_(2));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPDGravityController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!readStateInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianKinematics()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeGravityVector()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianPDGravityCommand()) {
    return controller_interface::return_type::ERROR;
  }

  publishDesiredCartesianState();
  publishCurrentCartesianState();
  publishCartesianErrorState();
  publishDebugState();

  if (!writeCommandInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void CartesianPDGravityController::desiredCartesianCallback(
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
    "Accepted desired Cartesian TCP position: [%.4f, %.4f, %.4f]",
    x_des_eig_(0),
    x_des_eig_(1),
    x_des_eig_(2));
}

bool CartesianPDGravityController::acceptDesiredCartesianState(
  const geometry_msgs::msg::PoseStamped & msg)
{
  x_des_eig_(0) = msg.pose.position.x;
  x_des_eig_(1) = msg.pose.position.y;
  x_des_eig_(2) = msg.pose.position.z;

  // Orientation is intentionally ignored in this first position-only controller.
  xdot_des_eig_.setZero();

  hold_initial_position_ = false;

  return true;
}

bool CartesianPDGravityController::readStateInterfaces()
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

bool CartesianPDGravityController::computeCartesianKinematics()
{
  return kinematics_adapter_.computeTcpKinematics(
    q_,
    qdot_,
    x_,
    xdot_,
    Jv_);
}

bool CartesianPDGravityController::computeGravityVector()
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

bool CartesianPDGravityController::computeCartesianPDGravityCommand()
{
  const auto n = joint_names_.size();

  if (Jv_.rows() != 3 || Jv_.cols() != static_cast<Eigen::Index>(n)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid translational Jacobian size. Expected 3x%zu, got %ldx%ld.",
      n,
      Jv_.rows(),
      Jv_.cols());

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

  for (int i = 0; i < 3; ++i) {
    f_cmd_(i) =
      kp_cartesian_[static_cast<size_t>(i)] * x_error_(i) +
      kd_cartesian_[static_cast<size_t>(i)] * xdot_error_(i);
  }

  tau_ = Jv_.transpose() * f_cmd_ + gravity_;

  q_error_debug_.setZero();
  qdot_error_debug_.setZero();

  return true;
}

void CartesianPDGravityController::publishDesiredCartesianState()
{
  if (!publish_desired_state_ || !desired_cartesian_pub_) {
    return;
  }

  fillPoseStampedPositionOnly(
    desired_cartesian_msg_,
    fixed_frame_,
    get_node()->now(),
    x_des_eig_);

  desired_cartesian_pub_->publish(desired_cartesian_msg_);
}

void CartesianPDGravityController::publishCurrentCartesianState()
{
  if (!publish_current_state_ || !current_cartesian_pub_) {
    return;
  }

  fillPoseStampedPositionOnly(
    current_cartesian_msg_,
    fixed_frame_,
    get_node()->now(),
    x_);

  current_cartesian_pub_->publish(current_cartesian_msg_);
}

void CartesianPDGravityController::publishCartesianErrorState()
{
  if (!publish_error_state_ || !cartesian_error_pub_) {
    return;
  }

  cartesian_error_msg_.header.stamp = get_node()->now();
  cartesian_error_msg_.header.frame_id = fixed_frame_;

  cartesian_error_msg_.vector.x = x_error_(0);
  cartesian_error_msg_.vector.y = x_error_(1);
  cartesian_error_msg_.vector.z = x_error_(2);

  cartesian_error_pub_->publish(cartesian_error_msg_);
}

void CartesianPDGravityController::publishDebugState()
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

bool CartesianPDGravityController::writeCommandInterfaces()
{
  return write_vector_to_command_interfaces(
    command_interfaces_,
    joint_names_,
    tau_,
    get_node()->get_logger(),
    *get_node()->get_clock(),
    "effort");
}

bool CartesianPDGravityController::expandCartesianVector(
  const std::vector<double> & input,
  Eigen::Vector3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setZero();
    return true;
  }

  if (input.size() != 3) {
    std::cerr << "[CartesianPDGravityController] Parameter '"
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

void CartesianPDGravityController::fillPoseStampedPositionOnly(
  geometry_msgs::msg::PoseStamped & msg,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const Eigen::Vector3d & position)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);

  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
}

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::CartesianPDGravityController,
  controller_interface::ControllerInterface)