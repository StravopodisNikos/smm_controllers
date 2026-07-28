#include "smm_controllers/cartesian_space/cartesian_robust_adaptive_id_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{

std::vector<std::string> makeTaskComponentNames(const std::size_t task_dim)
{
  const std::vector<std::string> all_names = {"x", "y", "z", "rx", "ry", "rz"};

  if (task_dim == 0 || task_dim > all_names.size()) {
    throw std::runtime_error(
      "Invalid task dimension for Cartesian robust-adaptive controller.");
  }

  return std::vector<std::string>(
    all_names.begin(),
    all_names.begin() + static_cast<std::ptrdiff_t>(task_dim));
}

bool isFiniteVector(const Eigen::VectorXd & v)
{
  return v.allFinite();
}

bool isFiniteMatrix(const Eigen::MatrixXd & m)
{
  return m.allFinite();
}

Eigen::VectorXd makeTaskVector(
  const std::vector<double> & position_values,
  const std::vector<double> & orientation_values,
  const std::size_t task_dim,
  const std::string & field_name,
  const bool allow_empty_orientation = true)
{
  if (task_dim < 1 || task_dim > 6) {
    throw std::runtime_error(
      field_name + ": invalid task dimension. Expected 1..6.");
  }

  if (position_values.size() != 3) {
    throw std::runtime_error(
      field_name + ": position vector must have exactly 3 values.");
  }

  if (task_dim > 3 &&
      orientation_values.size() != 3 &&
      !(allow_empty_orientation && orientation_values.empty()))
  {
    throw std::runtime_error(
      field_name +
      ": orientation vector must have exactly 3 values, or be empty if allowed.");
  }

  Eigen::VectorXd out(static_cast<Eigen::Index>(task_dim));
  out.setZero();

  for (std::size_t i = 0; i < std::min<std::size_t>(3, task_dim); ++i) {
    out(static_cast<Eigen::Index>(i)) = position_values[i];
  }

  if (task_dim > 3 && !orientation_values.empty()) {
    for (std::size_t i = 3; i < task_dim; ++i) {
      out(static_cast<Eigen::Index>(i)) = orientation_values[i - 3];
    }
  }

  return out;
}

void fillJointStateEffortMessage(
  sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & names,
  const Eigen::VectorXd & values,
  const rclcpp::Time & stamp)
{
  msg.header.stamp = stamp;
  msg.name = names;

  msg.position.clear();
  msg.velocity.clear();

  msg.effort.resize(static_cast<std::size_t>(values.size()));
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    msg.effort[static_cast<std::size_t>(i)] = values(i);
  }
}

void fillJointStatePositionVelocityEffortMessage(
  sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & names,
  const Eigen::VectorXd & position_values,
  const Eigen::VectorXd & velocity_values,
  const Eigen::VectorXd & effort_values,
  const rclcpp::Time & stamp)
{
  msg.header.stamp = stamp;
  msg.name = names;

  msg.position.resize(static_cast<std::size_t>(position_values.size()));
  for (Eigen::Index i = 0; i < position_values.size(); ++i) {
    msg.position[static_cast<std::size_t>(i)] = position_values(i);
  }

  msg.velocity.resize(static_cast<std::size_t>(velocity_values.size()));
  for (Eigen::Index i = 0; i < velocity_values.size(); ++i) {
    msg.velocity[static_cast<std::size_t>(i)] = velocity_values(i);
  }

  msg.effort.resize(static_cast<std::size_t>(effort_values.size()));
  for (Eigen::Index i = 0; i < effort_values.size(); ++i) {
    msg.effort[static_cast<std::size_t>(i)] = effort_values(i);
  }
}

Eigen::VectorXd vectorFromParameter(
  const std::vector<double> & values,
  const std::size_t expected_size,
  const std::string & field_name)
{
  if (values.empty()) {
    return Eigen::VectorXd();
  }

  if (values.size() == 1) {
    Eigen::VectorXd out(static_cast<Eigen::Index>(expected_size));
    out.setConstant(values.front());
    return out;
  }

  if (values.size() != expected_size) {
    throw std::runtime_error(
      field_name + ": expected either 1 value or " +
      std::to_string(expected_size) + " values, got " +
      std::to_string(values.size()) + ".");
  }

  Eigen::VectorXd out(static_cast<Eigen::Index>(expected_size));
  for (std::size_t i = 0; i < expected_size; ++i) {
    out(static_cast<Eigen::Index>(i)) = values[i];
  }

  return out;
}

}  // namespace

namespace smm_controllers
{

// ============================================================================
// 1. Lifecycle construction and interface declarations
// ============================================================================

CartesianRobustAdaptiveInvDynController::
CartesianRobustAdaptiveInvDynController() = default;

controller_interface::InterfaceConfiguration
CartesianRobustAdaptiveInvDynController::command_interface_configuration() const
{
  std::vector<std::string> names = joint_names_;

  if (names.empty() && get_node()) {
    try {
      names = get_node()->get_parameter("joints").as_string_array();
    } catch (const std::exception &) {
      // The controller manager may call this during configuration.
      // If the parameter is unavailable, return an empty list and let
      // configuration fail cleanly later.
    }
  }

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : names) {
    config.names.push_back(
      joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return config;
}

controller_interface::InterfaceConfiguration
CartesianRobustAdaptiveInvDynController::state_interface_configuration() const
{
  std::vector<std::string> names = joint_names_;

  if (names.empty() && get_node()) {
    try {
      names = get_node()->get_parameter("joints").as_string_array();
    } catch (const std::exception &) {
      // See comment in command_interface_configuration().
    }
  }

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : names) {
    config.names.push_back(
      joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return config;
}

// ============================================================================
// 2. on_init(): declare all parameters accepted by the controller
// ============================================================================

controller_interface::CallbackReturn
CartesianRobustAdaptiveInvDynController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    auto_declare<std::vector<double>>("kp_position", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_position", std::vector<double>{});
    auto_declare<std::vector<double>>("kp_orientation", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("x_des", std::vector<double>{});
    auto_declare<std::vector<double>>("xdot_des", std::vector<double>{});
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

    auto_declare<std::string>(
      "operational_dynamics_method",
      "exact_with_damped_fallback");

    auto_declare<double>("operational_damping", 1.0e-3);
    auto_declare<double>("task_acceleration_scale", 1.0);

    auto_declare<double>("condition_soft_limit", 500.0);
    auto_declare<double>("condition_hard_limit", 2000.0);
    auto_declare<bool>("orientation_condition_scaling", true);

    auto_declare<std::vector<double>>("lambda_position", std::vector<double>{});
    auto_declare<std::vector<double>>("lambda_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("k1_position", std::vector<double>{});
    auto_declare<std::vector<double>>("k1_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("k2_position", std::vector<double>{});
    auto_declare<std::vector<double>>("k2_orientation", std::vector<double>{});

    auto_declare<double>("tanh_kappa", 5.0);

    auto_declare<bool>("adaptive_enabled", true);
    auto_declare<double>("adaptive_deadzone", 0.001);

    auto_declare<std::vector<double>>(
      "adaptive_gain_position", std::vector<double>{});
    auto_declare<std::vector<double>>(
      "adaptive_gain_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>(
      "adaptive_leakage_position", std::vector<double>{});
    auto_declare<std::vector<double>>(
      "adaptive_leakage_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>(
      "adaptive_rho_initial_position", std::vector<double>{});
    auto_declare<std::vector<double>>(
      "adaptive_rho_initial_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>(
      "adaptive_rho_min_position", std::vector<double>{});
    auto_declare<std::vector<double>>(
      "adaptive_rho_min_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>(
      "adaptive_rho_max_position", std::vector<double>{});
    auto_declare<std::vector<double>>(
      "adaptive_rho_max_orientation", std::vector<double>{});

    auto_declare<double>("effort_limit", 80.0);
    auto_declare<std::vector<double>>("joint_effort_limits", std::vector<double>{});

    auto_declare<bool>("enforce_velocity_limits", true);
    auto_declare<double>("default_velocity_limit", 4.0841);
    auto_declare<double>("velocity_soft_margin", 0.25);
    auto_declare<double>("velocity_brake_gain", 15.0);
    auto_declare<std::vector<double>>("joint_velocity_limits", std::vector<double>{});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception during CartesianRobustAdaptiveInvDynController::on_init(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 3. on_configure(): load parameters, initialize adapters, create publishers
// ============================================================================

controller_interface::CallbackReturn
CartesianRobustAdaptiveInvDynController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  try {
    joint_names_ =
      get_node()->get_parameter("joints").as_string_array();

    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
      return controller_interface::CallbackReturn::ERROR;
    }

    const std::size_t dof = joint_names_.size();

    if (dof == 0 || dof > 6) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid controller DOF=%zu. Expected 1..6.", dof);
      return controller_interface::CallbackReturn::ERROR;
    }

    task_component_names_ = makeTaskComponentNames(dof);

    kp_position_ =
      get_node()->get_parameter("kp_position").as_double_array();
    kd_position_ =
      get_node()->get_parameter("kd_position").as_double_array();
    kp_orientation_ =
      get_node()->get_parameter("kp_orientation").as_double_array();
    kd_orientation_ =
      get_node()->get_parameter("kd_orientation").as_double_array();

    x_des_ =
      get_node()->get_parameter("x_des").as_double_array();
    orientation_des_ =
      get_node()->get_parameter("orientation_des").as_double_array();

    const auto xdot_des_param =
      get_node()->get_parameter("xdot_des").as_double_array();

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

    operational_dynamics_method_ =
      get_node()->get_parameter("operational_dynamics_method").as_string();
    operational_damping_ =
      get_node()->get_parameter("operational_damping").as_double();

    task_acceleration_scale_ =
      get_node()->get_parameter("task_acceleration_scale").as_double();

    condition_soft_limit_ =
      get_node()->get_parameter("condition_soft_limit").as_double();
    condition_hard_limit_ =
      get_node()->get_parameter("condition_hard_limit").as_double();
    orientation_condition_scaling_ =
      get_node()->get_parameter("orientation_condition_scaling").as_bool();

    tanh_kappa_ =
      get_node()->get_parameter("tanh_kappa").as_double();

    adaptive_enabled_ =
      get_node()->get_parameter("adaptive_enabled").as_bool();
    adaptive_deadzone_ =
      get_node()->get_parameter("adaptive_deadzone").as_double();

    effort_limit_ =
      get_node()->get_parameter("effort_limit").as_double();

    enforce_velocity_limits_ =
      get_node()->get_parameter("enforce_velocity_limits").as_bool();
    default_velocity_limit_ =
      get_node()->get_parameter("default_velocity_limit").as_double();
    velocity_soft_margin_ =
      get_node()->get_parameter("velocity_soft_margin").as_double();
    velocity_brake_gain_ =
      get_node()->get_parameter("velocity_brake_gain").as_double();

    if (!kinematics_adapter_.initialize(kinematics_data_dir_)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to initialize SmmKinematicsAdapter with directory '%s'.",
        kinematics_data_dir_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!dynamics_adapter_.initialize(
        dynamics_data_dir_,
        gravity_representation_,
        operational_dynamics_method_,
        operational_damping_))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to initialize SmmDynamicsAdapter with directory '%s'.",
        dynamics_data_dir_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const int adapter_dof = kinematics_adapter_.dof();
    if (adapter_dof != static_cast<int>(dof)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "DOF mismatch. Controller joints=%zu, kinematics adapter DOF=%d.",
        dof,
        adapter_dof);
      return controller_interface::CallbackReturn::ERROR;
    }

    lambda_ = makeTaskVector(
      get_node()->get_parameter("lambda_position").as_double_array(),
      get_node()->get_parameter("lambda_orientation").as_double_array(),
      dof,
      "lambda");

    k1_ = makeTaskVector(
      get_node()->get_parameter("k1_position").as_double_array(),
      get_node()->get_parameter("k1_orientation").as_double_array(),
      dof,
      "k1");

    k2_ = makeTaskVector(
      get_node()->get_parameter("k2_position").as_double_array(),
      get_node()->get_parameter("k2_orientation").as_double_array(),
      dof,
      "k2");

    adaptive_gain_ = makeTaskVector(
      get_node()->get_parameter("adaptive_gain_position").as_double_array(),
      get_node()->get_parameter("adaptive_gain_orientation").as_double_array(),
      dof,
      "adaptive_gain");

    adaptive_leakage_ = makeTaskVector(
      get_node()->get_parameter("adaptive_leakage_position").as_double_array(),
      get_node()->get_parameter("adaptive_leakage_orientation").as_double_array(),
      dof,
      "adaptive_leakage");

    adaptive_rho_ = makeTaskVector(
      get_node()->get_parameter("adaptive_rho_initial_position").as_double_array(),
      get_node()->get_parameter("adaptive_rho_initial_orientation").as_double_array(),
      dof,
      "adaptive_rho_initial");

    adaptive_rho_min_ = makeTaskVector(
      get_node()->get_parameter("adaptive_rho_min_position").as_double_array(),
      get_node()->get_parameter("adaptive_rho_min_orientation").as_double_array(),
      dof,
      "adaptive_rho_min");

    adaptive_rho_max_ = makeTaskVector(
      get_node()->get_parameter("adaptive_rho_max_position").as_double_array(),
      get_node()->get_parameter("adaptive_rho_max_orientation").as_double_array(),
      dof,
      "adaptive_rho_max");

    for (Eigen::Index i = 0; i < adaptive_rho_.size(); ++i) {
      if (adaptive_rho_min_(i) > adaptive_rho_max_(i)) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Invalid adaptive rho bounds at index %ld: min=%f > max=%f.",
          static_cast<long>(i),
          adaptive_rho_min_(i),
          adaptive_rho_max_(i));
        return controller_interface::CallbackReturn::ERROR;
      }

      adaptive_rho_(i) = std::clamp(
        adaptive_rho_(i),
        adaptive_rho_min_(i),
        adaptive_rho_max_(i));
    }

    joint_effort_limits_ = vectorFromParameter(
      get_node()->get_parameter("joint_effort_limits").as_double_array(),
      dof,
      "joint_effort_limits");

    if (joint_effort_limits_.size() == 0) {
      joint_effort_limits_.resize(static_cast<Eigen::Index>(dof));
      joint_effort_limits_.setConstant(effort_limit_);
    }

    joint_velocity_limits_ = vectorFromParameter(
      get_node()->get_parameter("joint_velocity_limits").as_double_array(),
      dof,
      "joint_velocity_limits");

    if (joint_velocity_limits_.size() == 0) {
      joint_velocity_limits_.resize(static_cast<Eigen::Index>(dof));
      joint_velocity_limits_.setConstant(default_velocity_limit_);
    }

    q_.resize(static_cast<Eigen::Index>(dof));
    qdot_.resize(static_cast<Eigen::Index>(dof));
    gravity_.resize(static_cast<Eigen::Index>(dof));
    tau_task_.resize(static_cast<Eigen::Index>(dof));
    tau_.resize(static_cast<Eigen::Index>(dof));
    q_error_debug_.resize(static_cast<Eigen::Index>(dof));
    qdot_error_debug_.resize(static_cast<Eigen::Index>(dof));

    q_.setZero();
    qdot_.setZero();
    gravity_.setZero();
    tau_task_.setZero();
    tau_.setZero();
    q_error_debug_.setZero();
    qdot_error_debug_.setZero();

    operational_error_.resize(static_cast<Eigen::Index>(dof));
    operational_velocity_error_.resize(static_cast<Eigen::Index>(dof));
    operational_acceleration_ref_.resize(static_cast<Eigen::Index>(dof));
    operational_wrench_.resize(static_cast<Eigen::Index>(dof));

    sliding_variable_.resize(static_cast<Eigen::Index>(dof));
    nominal_wrench_.resize(static_cast<Eigen::Index>(dof));
    robust_wrench_.resize(static_cast<Eigen::Index>(dof));
    adaptive_wrench_.resize(static_cast<Eigen::Index>(dof));

    operational_error_.setZero();
    operational_velocity_error_.setZero();
    operational_acceleration_ref_.setZero();
    operational_wrench_.setZero();

    sliding_variable_.setZero();
    nominal_wrench_.setZero();
    robust_wrench_.setZero();
    adaptive_wrench_.setZero();

    x_.setZero();
    xdot_.setZero();
    R_.setIdentity();
    omega_.setZero();

    x_des_eig_.setZero();
    xdot_des_eig_.setZero();
    R_des_.setIdentity();
    omega_des_.setZero();

    x_error_.setZero();
    xdot_error_.setZero();
    orientation_error_.setZero();
    omega_error_.setZero();

    force_cmd_.setZero();
    moment_cmd_.setZero();
    wrench_cmd_.resize(static_cast<Eigen::Index>(dof));
    wrench_cmd_.setZero();

    if (!x_des_.empty()) {
      if (!expandVector3(x_des_, x_des_eig_, "x_des")) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid x_des parameter.");
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    if (!xdot_des_param.empty()) {
      if (!expandVector3(xdot_des_param, xdot_des_eig_, "xdot_des")) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid xdot_des parameter.");
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    if (!orientation_des_.empty()) {
      if (!quaternionVectorToRotation(
          orientation_des_,
          R_des_,
          "orientation_des"))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Invalid orientation_des parameter.");
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    desired_cartesian_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/desired_cartesian_state",
        rclcpp::SystemDefaultsQoS(),
        std::bind(
          &CartesianRobustAdaptiveInvDynController::desiredCartesianCallback,
          this,
          std::placeholders::_1));

    desired_cartesian_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/desired_cartesian_state",
        rclcpp::SystemDefaultsQoS());

    current_cartesian_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_cartesian_state",
        rclcpp::SystemDefaultsQoS());

    position_error_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/position_error_state",
        rclcpp::SystemDefaultsQoS());

    orientation_error_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/orientation_error_state",
        rclcpp::SystemDefaultsQoS());

    error_state_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/cartesian_error_state",
        rclcpp::SystemDefaultsQoS());

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

    sliding_variable_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/sliding_variable",
        rclcpp::SystemDefaultsQoS());

    nominal_wrench_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/nominal_wrench",
        rclcpp::SystemDefaultsQoS());

    robust_wrench_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/robust_wrench",
        rclcpp::SystemDefaultsQoS());

    adaptive_wrench_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/adaptive_wrench",
        rclcpp::SystemDefaultsQoS());

    adaptive_rho_pub_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/adaptive_rho",
        rclcpp::SystemDefaultsQoS());

    q_error_pubs_.clear();
    q_error_msgs_.clear();

    for (std::size_t i = 0; i < dof; ++i) {
      q_error_pubs_.push_back(
        get_node()->create_publisher<std_msgs::msg::Float64>(
          "~/q_error_" + std::to_string(i),
          rclcpp::SystemDefaultsQoS()));

      q_error_msgs_.emplace_back();
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured CartesianRobustAdaptiveInvDynController with DOF=%zu, "
      "adaptive_enabled=%s, dynamics_method='%s', operational_damping=%g.",
      dof,
      adaptive_enabled_ ? "true" : "false",
      operational_dynamics_method_.c_str(),
      operational_damping_);

    std::ostringstream limits_oss;
    limits_oss << "Joint effort limits: ";
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      limits_oss
        << joint_names_[i]
        << "="
        << joint_effort_limits_(static_cast<Eigen::Index>(i))
        << " ";
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "%s",
      limits_oss.str().c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception during on_configure(): %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 4. on_activate(): initialize desired state and command buffers
// ============================================================================

controller_interface::CallbackReturn
CartesianRobustAdaptiveInvDynController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu effort command interfaces, got %zu.",
      joint_names_.size(),
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 2 * joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu.",
      2 * joint_names_.size(),
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readStateInterfaces()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to read state interfaces during activation.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!computeCartesianPoseKinematics()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to compute initial Cartesian pose during activation.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (hold_initial_position_ && x_des_.empty()) {
    x_des_eig_ = x_;
    xdot_des_eig_.setZero();
    R_des_ = R_;
    omega_des_.setZero();

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Holding initial TCP pose as desired reference.");
  }

  tau_.setZero();
  tau_task_.setZero();
  gravity_.setZero();

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!command_interfaces_[i].set_value(0.0)) {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "Failed to initialize command interface '%s' to zero.",
            command_interfaces_[i].get_name().c_str());
    }
  }

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "command_interfaces_[%zu] = %s ; expected joint_names_[%zu] = %s",
      i,
      command_interfaces_[i].get_name().c_str(),
      i,
      joint_names_[i].c_str());
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 5. update(): real-time control step
// ============================================================================

controller_interface::return_type
CartesianRobustAdaptiveInvDynController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  (void)time;

  update_period_sec_ = period.seconds();

  if (!readStateInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianPoseKinematics()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianInvDynCommand()) {
    tau_.setZero();
  }

  if (!writeCommandInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (publish_desired_state_) {
    publishDesiredCartesianState();
  }

  if (publish_current_state_) {
    publishCurrentCartesianState();
  }

  if (publish_error_state_) {
    publishCartesianErrorState();
  }

  if (publish_full_debug_state_) {
    publishFullDebugState();
    publishRobustAdaptiveDebugState();
  }

  return controller_interface::return_type::OK;
}

// ============================================================================
// 6. Reference handling
// ============================================================================

void CartesianRobustAdaptiveInvDynController::desiredCartesianCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!acceptDesiredCartesianState(*msg)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Rejected desired Cartesian state.");
  }
}

bool CartesianRobustAdaptiveInvDynController::acceptDesiredCartesianState(
  const geometry_msgs::msg::PoseStamped & msg)
{
  const auto & p = msg.pose.position;
  const auto & q = msg.pose.orientation;

  if (!std::isfinite(p.x) ||
      !std::isfinite(p.y) ||
      !std::isfinite(p.z) ||
      !std::isfinite(q.x) ||
      !std::isfinite(q.y) ||
      !std::isfinite(q.z) ||
      !std::isfinite(q.w))
  {
    return false;
  }

  Eigen::Quaterniond q_des(q.w, q.x, q.y, q.z);

  if (q_des.norm() < 1.0e-9) {
    return false;
  }

  q_des.normalize();

  x_des_eig_ << p.x, p.y, p.z;
  R_des_ = q_des.toRotationMatrix();

  xdot_des_eig_.setZero();
  omega_des_.setZero();

  return true;
}

// ============================================================================
// 7. State reading and kinematics
// ============================================================================

bool CartesianRobustAdaptiveInvDynController::readStateInterfaces()
{
  if (state_interfaces_.size() != 2 * joint_names_.size()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "State interface size mismatch. Expected %zu, got %zu.",
      2 * joint_names_.size(),
      state_interfaces_.size());
    return false;
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    // ROS 2 Kilted deprecates get_value().
    // get_optional() safely returns std::optional<double>, allowing us to detect
    // invalid or unavailable state-interface values without using deprecated API.
    const auto q_value =
      state_interfaces_[2 * i].get_optional();

    const auto qdot_value =
      state_interfaces_[2 * i + 1].get_optional();

    if (!q_value.has_value() || !qdot_value.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Failed to read state interfaces for joint '%s'.",
        joint_names_[i].c_str());
      return false;
    }

    q_(static_cast<Eigen::Index>(i)) = q_value.value();
    qdot_(static_cast<Eigen::Index>(i)) = qdot_value.value();
  }

  if (!q_.allFinite() || !qdot_.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Non-finite joint state detected.");
    return false;
  }

  return true;
}

bool CartesianRobustAdaptiveInvDynController::computeCartesianPoseKinematics()
{
  if (!kinematics_adapter_.computeTcpPoseKinematics(
      q_,
      qdot_,
      x_,      // TCP position
      R_,      // TCP orientation
      xdot_,   // TCP linear velocity
      omega_,  // TCP angular velocity
      Jop_))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute TCP pose kinematics.");
    return false;
  }

  // Cartesian tracking errors using the controller convention:
  // e = desired - current.
  x_error_ = x_des_eig_ - x_;
  xdot_error_ = xdot_des_eig_ - xdot_;

  // Orientation error:
  // e_R = vee(0.5 * (R_des*R^T - R*R_des^T)).
  orientation_error_ = computeOrientationError(R_des_, R_);

  // Angular velocity tracking error.
  omega_error_ = omega_des_ - omega_;

  computeJacobianColumnNorms();

  return true;
}

// ============================================================================
// 8. Operational-space inverse dynamics and robust-adaptive law
// ============================================================================

bool CartesianRobustAdaptiveInvDynController::computeCartesianInvDynCommand()
{
  const std::size_t task_dim = joint_names_.size();

  if (!dynamics_adapter_.computeNonredundantOperationalDynamics(
      q_,
      qdot_,
      Mx_,
      Cx_,
      Gx_,
      Jx_,
      dJx_))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to compute nonredundant operational-space dynamics.");
    return false;
  }

  if (Mx_.rows() != static_cast<Eigen::Index>(task_dim) ||
      Mx_.cols() != static_cast<Eigen::Index>(task_dim) ||
      Cx_.rows() != static_cast<Eigen::Index>(task_dim) ||
      Gx_.rows() != static_cast<Eigen::Index>(task_dim) ||
      Jx_.rows() != static_cast<Eigen::Index>(task_dim) ||
      Jx_.cols() != static_cast<Eigen::Index>(task_dim))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid operational dynamics dimensions.");
    return false;
  }

  if (!isFiniteMatrix(Mx_) ||
      !isFiniteVector(Cx_) ||
      !isFiniteVector(Gx_) ||
      !isFiniteMatrix(Jx_))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Non-finite operational dynamics detected.");
    return false;
  }

  computeSquareJacobianConditioning();

  operational_error_.setZero();
  operational_velocity_error_.setZero();

  for (std::size_t i = 0; i < std::min<std::size_t>(3, task_dim); ++i) {
    operational_error_(static_cast<Eigen::Index>(i)) =
      x_error_(static_cast<Eigen::Index>(i));

    operational_velocity_error_(static_cast<Eigen::Index>(i)) =
      xdot_error_(static_cast<Eigen::Index>(i));
  }

  if (task_dim > 3) {
    for (std::size_t i = 3; i < task_dim; ++i) {
      operational_error_(static_cast<Eigen::Index>(i)) =
        orientation_error_(static_cast<Eigen::Index>(i - 3));

      operational_velocity_error_(static_cast<Eigen::Index>(i)) =
        omega_error_(static_cast<Eigen::Index>(i - 3));
    }
  }

  double orientation_scale = 1.0;

  if (orientation_condition_scaling_ && task_dim > 3) {
    if (jacobian_condition_ >= condition_hard_limit_) {
      orientation_scale = 0.0;
    } else if (jacobian_condition_ > condition_soft_limit_) {
      orientation_scale =
        (condition_hard_limit_ - jacobian_condition_) /
        (condition_hard_limit_ - condition_soft_limit_);
    }

    orientation_scale = std::clamp(orientation_scale, 0.0, 1.0);
  }

  operational_acceleration_ref_.setZero();

  for (std::size_t i = 0; i < task_dim; ++i) {
    double kp_i = 0.0;
    double kd_i = 0.0;

    if (i < 3) {
      kp_i = kp_position_[i];
      kd_i = kd_position_[i];
    } else {
      kp_i = kp_orientation_[i - 3];
      kd_i = kd_orientation_[i - 3];
    }

    double scale_i = task_acceleration_scale_;

    if (i >= 3) {
      scale_i *= orientation_scale;
    }

    // Reference operational acceleration:
    // a_ref_i = scale * (Kp_i * e_i + Kd_i * e_dot_i).
    // This is the commanded task-space acceleration used by inverse dynamics.
    operational_acceleration_ref_(static_cast<Eigen::Index>(i)) =
      scale_i *
      (
        kp_i * operational_error_(static_cast<Eigen::Index>(i)) +
        kd_i * operational_velocity_error_(static_cast<Eigen::Index>(i))
      );
  }

  // Sliding variable:
  // s = e_dot + Lambda * e.
  // This is the filtered tracking error used by the robust/adaptive law.
  sliding_variable_ =
    operational_velocity_error_ +
    lambda_.cwiseProduct(operational_error_);

  // Nominal operational-space inverse dynamics:
  // F_nom = Mx * a_ref + Cx + Gx.
  // This is the model-based wrench required to produce the reference acceleration.
  nominal_wrench_ =
    Mx_ * operational_acceleration_ref_ + Cx_ + Gx_;

  robust_wrench_.setZero();

  for (Eigen::Index i = 0; i < robust_wrench_.size(); ++i) {
    // Fixed robust RIDOSC term:
    // F_rob_i = K1_i*s_i + K2_i*tanh(kappa*s_i).
    // tanh(.) is a smooth sign(.) approximation and reduces chattering.
    robust_wrench_(i) =
      k1_(i) * sliding_variable_(i) +
      k2_(i) * std::tanh(tanh_kappa_ * sliding_variable_(i));
  }

  updateAdaptiveWrench(update_period_sec_);

  // Total operational wrench:
  // F = F_nom + F_rob + F_ad.
  // The adaptive term is added in operational space before torque mapping.
  operational_wrench_ =
    nominal_wrench_ + robust_wrench_ + adaptive_wrench_;

  if (!operational_wrench_.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Non-finite operational wrench detected.");
    return false;
  }

  wrench_cmd_ = operational_wrench_;

  force_cmd_.setZero();
  moment_cmd_.setZero();

  for (std::size_t i = 0; i < std::min<std::size_t>(3, task_dim); ++i) {
    force_cmd_(static_cast<Eigen::Index>(i)) =
      operational_wrench_(static_cast<Eigen::Index>(i));
  }

  if (task_dim > 3) {
    for (std::size_t i = 3; i < task_dim; ++i) {
      moment_cmd_(static_cast<Eigen::Index>(i - 3)) =
        operational_wrench_(static_cast<Eigen::Index>(i));
    }
  }

  // Joint torque mapping:
  // tau = Jx^T * F.
  // Converts the commanded operational wrench into active joint efforts.
  tau_ = Jx_.transpose() * operational_wrench_;

  tau_task_ = tau_;

  // Joint-space gravity debug contribution induced by the operational gravity vector.
  gravity_ = Jx_.transpose() * Gx_;

  applyVelocityLimitTorqueFilter(qdot_, tau_);

  for (Eigen::Index i = 0; i < tau_.size(); ++i) {
    if (!std::isfinite(tau_(i))) {
      tau_(i) = 0.0;
    }

    const double limit_i =
      joint_effort_limits_.size() == tau_.size()
      ? joint_effort_limits_(i)
      : effort_limit_;

    tau_(i) = std::clamp(tau_(i), -limit_i, limit_i);
  }

  return true;
}

void CartesianRobustAdaptiveInvDynController::updateAdaptiveWrench(
  const double dt)
{
  adaptive_wrench_.setZero();

  if (!adaptive_enabled_) {
    return;
  }

  if (dt <= 0.0 || !std::isfinite(dt)) {
    return;
  }

  if (adaptive_rho_.size() != sliding_variable_.size() ||
      adaptive_gain_.size() != sliding_variable_.size() ||
      adaptive_leakage_.size() != sliding_variable_.size() ||
      adaptive_rho_min_.size() != sliding_variable_.size() ||
      adaptive_rho_max_.size() != sliding_variable_.size())
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Adaptive wrench skipped because vector sizes are inconsistent.");
    return;
  }

  for (Eigen::Index i = 0; i < sliding_variable_.size(); ++i) {
    const double abs_s = std::abs(sliding_variable_(i));

    // Deadzone:
    // prevents adaptation from reacting to tiny numerical errors or noise.
    const double drive =
      std::max(abs_s - adaptive_deadzone_, 0.0);

    // Adaptive uncertainty-bound update:
    // rho_hat_dot = gamma * max(|s|-deadzone,0) - leakage*rho_hat.
    // Persistent sliding error increases the estimated uncertainty bound.
    // Leakage prevents parameter drift and slowly forgets unnecessary compensation.
    const double rho_dot =
      adaptive_gain_(i) * drive -
      adaptive_leakage_(i) * adaptive_rho_(i);

    adaptive_rho_(i) += dt * rho_dot;

    // Projection:
    // keeps the adaptive uncertainty estimate inside safe configured bounds.
    adaptive_rho_(i) = std::clamp(
      adaptive_rho_(i),
      adaptive_rho_min_(i),
      adaptive_rho_max_(i));

    // Adaptive robust wrench:
    // F_ad = rho_hat * tanh(kappa*s).
    // This compensates the estimated residual operational disturbance
    // in each task-space direction.
    adaptive_wrench_(i) =
      adaptive_rho_(i) *
      std::tanh(tanh_kappa_ * sliding_variable_(i));
  }
}

// ============================================================================
// 9. Jacobian diagnostics
// ============================================================================

bool CartesianRobustAdaptiveInvDynController::computeSquareJacobianConditioning()
{
  if (Jx_.rows() == 0 || Jx_.cols() == 0) {
    jacobian_condition_ = std::numeric_limits<double>::infinity();
    singular_values_.resize(0);
    return false;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jx_);
  singular_values_ = svd.singularValues();

  if (singular_values_.size() == 0) {
    jacobian_condition_ = std::numeric_limits<double>::infinity();
    return false;
  }

  const double sigma_max = singular_values_(0);
  const double sigma_min = singular_values_(singular_values_.size() - 1);

  if (sigma_min <= 1.0e-12) {
    jacobian_condition_ = std::numeric_limits<double>::infinity();
  } else {
    jacobian_condition_ = sigma_max / sigma_min;
  }

  return std::isfinite(jacobian_condition_);
}

void CartesianRobustAdaptiveInvDynController::computeJacobianConditioning()
{
  computeSquareJacobianConditioning();
}

void CartesianRobustAdaptiveInvDynController::computeJacobianColumnNorms()
{
  if (Jop_.cols() == 0) {
    jacobian_column_norms_.resize(0);
    return;
  }

  jacobian_column_norms_.resize(Jop_.cols());

  for (Eigen::Index j = 0; j < Jop_.cols(); ++j) {
    jacobian_column_norms_(j) = Jop_.col(j).norm();
  }
}

// ============================================================================
// 10. Safety filter and command writing
// ============================================================================

void CartesianRobustAdaptiveInvDynController::applyVelocityLimitTorqueFilter(
  const Eigen::VectorXd & qdot,
  Eigen::VectorXd & tau)
{
  if (!enforce_velocity_limits_) {
    return;
  }

  if (qdot.size() != tau.size() ||
      joint_velocity_limits_.size() != tau.size())
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Velocity-limit filter skipped because vector sizes are inconsistent.");
    return;
  }

  for (Eigen::Index i = 0; i < tau.size(); ++i) {
    const double v_limit = joint_velocity_limits_(i);

    if (v_limit <= 0.0) {
      continue;
    }

    const double margin =
      std::min(velocity_soft_margin_, 0.5 * v_limit);

    const double v_soft = v_limit - margin;
    const double v = qdot(i);
    const double abs_v = std::abs(v);

    if (abs_v <= v_soft) {
      continue;
    }

    const double direction = (v >= 0.0) ? 1.0 : -1.0;

    const double ratio =
      std::clamp((v_limit - abs_v) / margin, 0.0, 1.0);

    // If torque pushes further into the velocity limit, smoothly reduce it.
    if (tau(i) * direction > 0.0) {
      tau(i) *= ratio;
    }

    // Add braking torque once inside the soft-limit region.
    const double overspeed_soft = abs_v - v_soft;
    tau(i) -= velocity_brake_gain_ * overspeed_soft * direction;
  }
}

bool CartesianRobustAdaptiveInvDynController::writeCommandInterfaces()
{
  if (command_interfaces_.size() != static_cast<std::size_t>(tau_.size())) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Command interface size mismatch. Expected %ld, got %zu.",
      static_cast<long>(tau_.size()),
      command_interfaces_.size());
    return false;
  }

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (!command_interfaces_[i].set_value(tau_(static_cast<Eigen::Index>(i)))) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Failed to write effort command for joint '%s'.",
            joint_names_[i].c_str());
        return false;
    }
  }

  return true;
}

// ============================================================================
// 11. Standard publishers
// ============================================================================

void CartesianRobustAdaptiveInvDynController::publishDesiredCartesianState()
{
  fillPoseStamped(
    desired_cartesian_msg_,
    fixed_frame_,
    get_node()->now(),
    x_des_eig_,
    R_des_);

  desired_cartesian_pub_->publish(desired_cartesian_msg_);
}

void CartesianRobustAdaptiveInvDynController::publishCurrentCartesianState()
{
  fillPoseStamped(
    current_cartesian_msg_,
    fixed_frame_,
    get_node()->now(),
    x_,
    R_);

  current_cartesian_pub_->publish(current_cartesian_msg_);
}

void CartesianRobustAdaptiveInvDynController::publishCartesianErrorState()
{
  const rclcpp::Time stamp = get_node()->now();

  position_error_msg_.header.stamp = stamp;
  position_error_msg_.header.frame_id = fixed_frame_;
  position_error_msg_.vector.x = x_error_.x();
  position_error_msg_.vector.y = x_error_.y();
  position_error_msg_.vector.z = x_error_.z();
  position_error_pub_->publish(position_error_msg_);

  orientation_error_msg_.header.stamp = stamp;
  orientation_error_msg_.header.frame_id = fixed_frame_;
  orientation_error_msg_.vector.x = orientation_error_.x();
  orientation_error_msg_.vector.y = orientation_error_.y();
  orientation_error_msg_.vector.z = orientation_error_.z();
  orientation_error_pub_->publish(orientation_error_msg_);

  fillJointStatePositionVelocityEffortMessage(
    error_state_msg_,
    task_component_names_,
    operational_error_,
    operational_velocity_error_,
    sliding_variable_,
    stamp);

  error_state_pub_->publish(error_state_msg_);

  for (std::size_t i = 0; i < q_error_pubs_.size(); ++i) {
    if (i < static_cast<std::size_t>(operational_error_.size())) {
      q_error_msgs_[i].data =
        operational_error_(static_cast<Eigen::Index>(i));
      q_error_pubs_[i]->publish(q_error_msgs_[i]);
    }
  }
}

void CartesianRobustAdaptiveInvDynController::publishDebugState()
{
  publishCartesianErrorState();
}

void CartesianRobustAdaptiveInvDynController::publishFullDebugState()
{
  const rclcpp::Time stamp = get_node()->now();

  task_force_msg_.header.stamp = stamp;
  task_force_msg_.header.frame_id = fixed_frame_;
  task_force_msg_.vector.x = force_cmd_.x();
  task_force_msg_.vector.y = force_cmd_.y();
  task_force_msg_.vector.z = force_cmd_.z();
  task_force_pub_->publish(task_force_msg_);

  task_moment_msg_.header.stamp = stamp;
  task_moment_msg_.header.frame_id = fixed_frame_;
  task_moment_msg_.vector.x = moment_cmd_.x();
  task_moment_msg_.vector.y = moment_cmd_.y();
  task_moment_msg_.vector.z = moment_cmd_.z();
  task_moment_pub_->publish(task_moment_msg_);

  fillJointStateEffortMessage(
    task_torque_msg_,
    joint_names_,
    tau_task_,
    stamp);
  task_torque_pub_->publish(task_torque_msg_);

  fillJointStateEffortMessage(
    gravity_torque_msg_,
    joint_names_,
    gravity_,
    stamp);
  gravity_torque_pub_->publish(gravity_torque_msg_);

  fillJointStateEffortMessage(
    commanded_torque_msg_,
    joint_names_,
    tau_,
    stamp);
  commanded_torque_pub_->publish(commanded_torque_msg_);

  jacobian_condition_msg_.data = jacobian_condition_;
  jacobian_condition_pub_->publish(jacobian_condition_msg_);

  fillJointStateEffortMessage(
    jacobian_singular_values_msg_,
    task_component_names_,
    singular_values_,
    stamp);
  jacobian_singular_values_pub_->publish(jacobian_singular_values_msg_);

  fillJointStateEffortMessage(
    jacobian_column_norms_msg_,
    joint_names_,
    jacobian_column_norms_,
    stamp);
  jacobian_column_norms_pub_->publish(jacobian_column_norms_msg_);
}

void CartesianRobustAdaptiveInvDynController::publishRobustAdaptiveDebugState()
{
  const rclcpp::Time stamp = get_node()->now();

  fillJointStateEffortMessage(
    sliding_variable_msg_,
    task_component_names_,
    sliding_variable_,
    stamp);
  sliding_variable_pub_->publish(sliding_variable_msg_);

  fillJointStateEffortMessage(
    nominal_wrench_msg_,
    task_component_names_,
    nominal_wrench_,
    stamp);
  nominal_wrench_pub_->publish(nominal_wrench_msg_);

  fillJointStateEffortMessage(
    robust_wrench_msg_,
    task_component_names_,
    robust_wrench_,
    stamp);
  robust_wrench_pub_->publish(robust_wrench_msg_);

  fillJointStateEffortMessage(
    adaptive_wrench_msg_,
    task_component_names_,
    adaptive_wrench_,
    stamp);
  adaptive_wrench_pub_->publish(adaptive_wrench_msg_);

  fillJointStateEffortMessage(
    adaptive_rho_msg_,
    task_component_names_,
    adaptive_rho_,
    stamp);
  adaptive_rho_pub_->publish(adaptive_rho_msg_);
}

// ============================================================================
// 12. Utility helpers
// ============================================================================

bool CartesianRobustAdaptiveInvDynController::expandVector3(
  const std::vector<double> & input,
  Eigen::Vector3d & output,
  const std::string & field_name)
{
  if (input.size() != 3) {
    (void)field_name;
    return false;
  }

  output << input[0], input[1], input[2];
  return output.allFinite();
}

bool CartesianRobustAdaptiveInvDynController::quaternionVectorToRotation(
  const std::vector<double> & input,
  Eigen::Matrix3d & output,
  const std::string & field_name)
{
  if (input.size() != 4) {
    (void)field_name;
    return false;
  }

  Eigen::Quaterniond q(input[3], input[0], input[1], input[2]);

  if (q.norm() < 1.0e-9 || !std::isfinite(q.norm())) {
    return false;
  }

  q.normalize();
  output = q.toRotationMatrix();

  return output.allFinite();
}

Eigen::Quaterniond CartesianRobustAdaptiveInvDynController::rotationToQuaternion(
  const Eigen::Matrix3d & R)
{
  Eigen::Quaterniond q(R);
  q.normalize();
  return q;
}

Eigen::Vector3d CartesianRobustAdaptiveInvDynController::computeOrientationError(
  const Eigen::Matrix3d & R_des,
  const Eigen::Matrix3d & R)
{
  // Orientation error:
  // e_R = vee(0.5 * (R_des*R^T - R*R_des^T)).
  // This gives a compact axis-error vector consistent with operational-space
  // rotational feedback.
  const Eigen::Matrix3d skew_error =
    0.5 * (R_des * R.transpose() - R * R_des.transpose());

  Eigen::Vector3d e;
  e << skew_error(2, 1),
       skew_error(0, 2),
       skew_error(1, 0);

  return e;
}

void CartesianRobustAdaptiveInvDynController::fillPoseStamped(
  geometry_msgs::msg::PoseStamped & msg,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const Eigen::Vector3d & position,
  const Eigen::Matrix3d & orientation)
{
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();

  const Eigen::Quaterniond q = rotationToQuaternion(orientation);

  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
}

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::CartesianRobustAdaptiveInvDynController,
  controller_interface::ControllerInterface)