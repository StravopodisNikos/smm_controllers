#include "smm_controllers/cartesian_space/cartesian_robust_id_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

CartesianRobustInvDynController::CartesianRobustInvDynController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CartesianRobustInvDynController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});

    auto_declare<std::vector<double>>("kp_position", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_position", std::vector<double>{});

    auto_declare<std::vector<double>>("kp_orientation", std::vector<double>{});
    auto_declare<std::vector<double>>("kd_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("lambda_position", std::vector<double>{});
    auto_declare<std::vector<double>>("lambda_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("k1_position", std::vector<double>{});
    auto_declare<std::vector<double>>("k1_orientation", std::vector<double>{});

    auto_declare<std::vector<double>>("k2_position", std::vector<double>{});
    auto_declare<std::vector<double>>("k2_orientation", std::vector<double>{});

    auto_declare<double>("tanh_kappa", 5.0);
    auto_declare<double>("effort_limit", 80.0);

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

    auto_declare<double>("task_acceleration_scale", 1.0);
    auto_declare<double>("condition_soft_limit", 500.0);
    auto_declare<double>("condition_hard_limit", 2000.0);
    auto_declare<bool>("orientation_condition_scaling", true);
    
    auto_declare<std::string>("operational_dynamics_method","exact_with_damped_fallback");
    auto_declare<double>("operational_damping",1.0e-3);

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
CartesianRobustInvDynController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
CartesianRobustInvDynController::state_interface_configuration() const
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

controller_interface::CallbackReturn CartesianRobustInvDynController::on_configure(
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

    task_acceleration_scale_ =
        get_node()->get_parameter("task_acceleration_scale").as_double();

    condition_soft_limit_ =
        get_node()->get_parameter("condition_soft_limit").as_double();

    condition_hard_limit_ =
        get_node()->get_parameter("condition_hard_limit").as_double();

    orientation_condition_scaling_ =
        get_node()->get_parameter("orientation_condition_scaling").as_bool();

    operational_dynamics_method_ =
        get_node()->get_parameter("operational_dynamics_method").as_string();

    operational_damping_ =
        get_node()->get_parameter("operational_damping").as_double();

    const auto lambda_position =
        get_node()->get_parameter("lambda_position").as_double_array();

    const auto lambda_orientation =
        get_node()->get_parameter("lambda_orientation").as_double_array();

    const auto k1_position =
        get_node()->get_parameter("k1_position").as_double_array();

    const auto k1_orientation =
        get_node()->get_parameter("k1_orientation").as_double_array();

    const auto k2_position =
        get_node()->get_parameter("k2_position").as_double_array();

    const auto k2_orientation =
        get_node()->get_parameter("k2_orientation").as_double_array();

    tanh_kappa_ =
        get_node()->get_parameter("tanh_kappa").as_double();

    effort_limit_ =
        get_node()->get_parameter("effort_limit").as_double();

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

    if (
        lambda_position.size() != 3 ||
        lambda_orientation.size() != 3 ||
        k1_position.size() != 3 ||
        k1_orientation.size() != 3 ||
        k2_position.size() != 3 ||
        k2_orientation.size() != 3)
        {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Robust Cartesian gains must all have exactly 3 values. "
            "lambda_position=%zu lambda_orientation=%zu k1_position=%zu "
            "k1_orientation=%zu k2_position=%zu k2_orientation=%zu",
            lambda_position.size(),
            lambda_orientation.size(),
            k1_position.size(),
            k1_orientation.size(),
            k2_position.size(),
            k2_orientation.size());

        return controller_interface::CallbackReturn::ERROR;
    }

    if (tanh_kappa_ <= 0.0) {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "tanh_kappa must be positive. Resetting to 5.0.");
        tanh_kappa_ = 5.0;
    }

    if (effort_limit_ <= 0.0) {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "effort_limit must be positive. Resetting to 80.0.");
        effort_limit_ = 80.0;
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

    wrench_cmd_.resize(static_cast<Eigen::Index>(n));

    Mx_.resize(n, n);
    Cx_.resize(n);
    Gx_.resize(n);
    Jx_.resize(n, n);
    dJx_.resize(n, n);

    operational_error_.resize(n);
    operational_velocity_error_.resize(n);
    operational_acceleration_ref_.resize(n);
    operational_wrench_.resize(n);

    lambda_.resize(static_cast<Eigen::Index>(n));
    k1_.resize(static_cast<Eigen::Index>(n));
    k2_.resize(static_cast<Eigen::Index>(n));

    sliding_variable_.resize(static_cast<Eigen::Index>(n));
    nominal_wrench_.resize(static_cast<Eigen::Index>(n));
    robust_wrench_.resize(static_cast<Eigen::Index>(n));

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

    Mx_.setZero();
    Cx_.setZero();
    Gx_.setZero();
    Jx_.setZero();
    dJx_.setZero();

    operational_error_.setZero();
    operational_velocity_error_.setZero();
    operational_acceleration_ref_.setZero();
    operational_wrench_.setZero();

    lambda_.setZero();
    k1_.setZero();
    k2_.setZero();

    for (std::size_t i = 0; i < n; ++i) {
        if (i < 3) {
            lambda_(static_cast<Eigen::Index>(i)) = lambda_position[i];
            k1_(static_cast<Eigen::Index>(i)) = k1_position[i];
            k2_(static_cast<Eigen::Index>(i)) = k2_position[i];
        } else {
            const std::size_t oi = i - 3;

            if (oi < 3) {
            lambda_(static_cast<Eigen::Index>(i)) = lambda_orientation[oi];
            k1_(static_cast<Eigen::Index>(i)) = k1_orientation[oi];
            k2_(static_cast<Eigen::Index>(i)) = k2_orientation[oi];
            }
        }
    }

    sliding_variable_.setZero();
    nominal_wrench_.setZero();
    robust_wrench_.setZero();

    if (!kinematics_adapter_.initialize(kinematics_data_dir_)) {
        RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to initialize SmmKinematicsAdapter with kinematics_data_dir='%s'",
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
        "Failed to initialize dynamics adapter.");

    return CallbackReturn::ERROR;
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
            &CartesianRobustInvDynController::desiredCartesianCallback,
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
        "Configured CartesianRobustInvDynController with %zu joints.", n);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Cartesian pose reference topic: ~/desired_cartesian_state");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianRobustInvDynController::on_activate(
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
      "CartesianRobustInvDynController holding initial TCP pose.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianRobustInvDynController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!readStateInterfaces()) {
    return controller_interface::return_type::ERROR;
  }

  if (!computeCartesianPoseKinematics()) {
    return controller_interface::return_type::ERROR;
  }

  // [16-7-26] Removed - it s too harsh for simulation
  //if (!computeCartesianInvDynCommand()) {
  //  return controller_interface::return_type::ERROR;
  //}

  if (!computeCartesianInvDynCommand()) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Cartesian inverse dynamics failed. Sending zero effort for this cycle.");

    tau_.setZero();
    tau_task_ = tau_;

    writeCommandInterfaces();

    return controller_interface::return_type::OK;
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

void CartesianRobustInvDynController::desiredCartesianCallback(
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

bool CartesianRobustInvDynController::acceptDesiredCartesianState(
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

bool CartesianRobustInvDynController::readStateInterfaces()
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

bool CartesianRobustInvDynController::computeCartesianPoseKinematics()
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

bool CartesianRobustInvDynController::computeSquareJacobianConditioning()
{
  if (
    Jx_.rows() == 0 ||
    Jx_.cols() == 0 ||
    Jx_.rows() != Jx_.cols())
  {
    jacobian_condition_ = 1.0e12;
    singular_values_.resize(0);
    return false;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jx_);
  singular_values_ = svd.singularValues();

  if (singular_values_.size() == 0) {
    jacobian_condition_ = 1.0e12;
    return false;
  }

  const double sigma_max = singular_values_(0);
  const double sigma_min = singular_values_(singular_values_.size() - 1);

  if (sigma_min <= 1.0e-9) {
    jacobian_condition_ = 1.0e12;
  } else {
    jacobian_condition_ = sigma_max / sigma_min;
  }

  return true;
}

bool CartesianRobustInvDynController::computeCartesianInvDynCommand()
{
  const auto n = joint_names_.size();

  if (n == 0) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "No joints configured.");
    return false;
  }

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

  if (
    Mx_.rows() != static_cast<Eigen::Index>(n) ||
    Mx_.cols() != static_cast<Eigen::Index>(n) ||
    Cx_.size() != static_cast<Eigen::Index>(n) ||
    Gx_.size() != static_cast<Eigen::Index>(n) ||
    Jx_.rows() != static_cast<Eigen::Index>(n) ||
    Jx_.cols() != static_cast<Eigen::Index>(n))
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Invalid operational dynamics dimensions.");
    return false;
  }

  computeSquareJacobianConditioning();
  computeJacobianColumnNorms();

  x_error_ = x_des_eig_ - x_;
  xdot_error_ = xdot_des_eig_ - xdot_;

  orientation_error_ = computeOrientationError(R_des_, R_);
  omega_error_ = omega_des_ - omega_;

  operational_error_.setZero();
  operational_velocity_error_.setZero();
  operational_acceleration_ref_.setZero();

  /*
   * Nonredundant task vector convention:
   *
   * DOF >= 1: x
   * DOF >= 2: y
   * DOF >= 3: z
   * DOF >= 4: orientation x
   * DOF >= 5: orientation y
   * DOF >= 6: orientation z
   *
   * This matches extractSquareOperationalJacobian(), which uses
   * Jop_full.topRows(dof).
   */

  if (n >= 1) {
    operational_error_(0) = x_error_(0);
    operational_velocity_error_(0) = xdot_error_(0);
  }

  if (n >= 2) {
    operational_error_(1) = x_error_(1);
    operational_velocity_error_(1) = xdot_error_(1);
  }

  if (n >= 3) {
    operational_error_(2) = x_error_(2);
    operational_velocity_error_(2) = xdot_error_(2);
  }

  if (n >= 4) {
    operational_error_(3) = orientation_error_(0);
    operational_velocity_error_(3) = omega_error_(0);
  }

  if (n >= 5) {
    operational_error_(4) = orientation_error_(1);
    operational_velocity_error_(4) = omega_error_(1);
  }

  if (n >= 6) {
    operational_error_(5) = orientation_error_(2);
    operational_velocity_error_(5) = omega_error_(2);
  }

  for (std::size_t i = 0; i < n; ++i) {
    double kp = 0.0;
    double kd = 0.0;

    if (i < 3) {
      kp = kp_position_[i];
      kd = kd_position_[i];
    } else {
      const std::size_t oi = i - 3;
      kp = kp_orientation_[oi];
      kd = kd_orientation_[oi];
    }

    operational_acceleration_ref_(static_cast<Eigen::Index>(i)) =
      task_acceleration_scale_ *
      (
        kp * operational_error_(static_cast<Eigen::Index>(i)) +
        kd * operational_velocity_error_(static_cast<Eigen::Index>(i))
      );
  }

  /*
   * Singularity protection:
   * keep position active, reduce orientation task near singularity.
   */
  if (orientation_condition_scaling_ && n > 3) {
    double orientation_scale = 1.0;

    if (jacobian_condition_ >= condition_hard_limit_) {
      orientation_scale = 0.0;
    } else if (jacobian_condition_ > condition_soft_limit_) {
      orientation_scale =
        (condition_hard_limit_ - jacobian_condition_) /
        (condition_hard_limit_ - condition_soft_limit_);
    }

    orientation_scale = std::clamp(orientation_scale, 0.0, 1.0);

    for (std::size_t i = 3; i < n; ++i) {
      operational_acceleration_ref_(static_cast<Eigen::Index>(i)) *=
        orientation_scale;
    }
  }

  /*
   * Cartesian robust inverse dynamics:
   *
   *    s = edot + lambda * e
   *    F_nominal = Mx * ar + Cx + Gx
   *    F_robust = K1 * s + K2 * tanh(kappa * s)
   *    F_total = F_nominal + F_robust
   *    tau = Jxᵀ * F_total
   * 
   */
    sliding_variable_ =
    operational_velocity_error_ +
    lambda_.cwiseProduct(operational_error_);

    nominal_wrench_ =
    Mx_ * operational_acceleration_ref_ + Cx_ + Gx_;

    robust_wrench_.resize(operational_wrench_.size());
    robust_wrench_.setZero();

    for (Eigen::Index i = 0; i < operational_wrench_.size(); ++i) {
    const double s_i = sliding_variable_(i);

    robust_wrench_(i) =
        k1_(i) * s_i +
        k2_(i) * std::tanh(tanh_kappa_ * s_i);
    }

    operational_wrench_ =
    nominal_wrench_ + robust_wrench_;

    tau_ =
    Jx_.transpose() * operational_wrench_;

    for (Eigen::Index i = 0; i < tau_.size(); ++i) {
    if (!std::isfinite(tau_(i))) {
        tau_(i) = 0.0;
    }

    tau_(i) = std::clamp(
        tau_(i),
        -effort_limit_,
        effort_limit_);
    }

    tau_task_ = tau_;

  /*
   * Keep old force/moment debug topics compatible.
   */
  force_cmd_.setZero();
  moment_cmd_.setZero();

  const Eigen::Index force_size =
    std::min<Eigen::Index>(3, operational_wrench_.size());

  for (Eigen::Index i = 0; i < force_size; ++i) {
    force_cmd_(i) = operational_wrench_(i);
  }

  if (operational_wrench_.size() > 3) {
    const Eigen::Index moment_size =
      std::min<Eigen::Index>(3, operational_wrench_.size() - 3);

    for (Eigen::Index i = 0; i < moment_size; ++i) {
      moment_cmd_(i) = operational_wrench_(3 + i);
    }
  }

  /*
   * Optional: compute joint gravity only for the old gravity_torque debug topic.
   * This is not used in the commanded torque.
   */
  (void)dynamics_adapter_.computeGravity(q_, qdot_, gravity_);

  q_error_debug_.setZero();
  qdot_error_debug_.setZero();

  return true;
}

void CartesianRobustInvDynController::computeJacobianConditioning()
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

void CartesianRobustInvDynController::computeJacobianColumnNorms()
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

void CartesianRobustInvDynController::publishDesiredCartesianState()
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

void CartesianRobustInvDynController::publishCurrentCartesianState()
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

void CartesianRobustInvDynController::publishCartesianErrorState()
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

void CartesianRobustInvDynController::publishDebugState()
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

void CartesianRobustInvDynController::publishFullDebugState()
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

bool CartesianRobustInvDynController::writeCommandInterfaces()
{
  return write_vector_to_command_interfaces(
    command_interfaces_,
    joint_names_,
    tau_,
    get_node()->get_logger(),
    *get_node()->get_clock(),
    "effort");
}

bool CartesianRobustInvDynController::expandVector3(
  const std::vector<double> & input,
  Eigen::Vector3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setZero();
    return true;
  }

  if (input.size() != 3) {
    std::cerr << "[CartesianRobustInvDynController] Parameter '"
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

bool CartesianRobustInvDynController::quaternionVectorToRotation(
  const std::vector<double> & input,
  Eigen::Matrix3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setIdentity();
    return true;
  }

  if (input.size() != 4) {
    std::cerr << "[CartesianRobustInvDynController] Parameter '"
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
    std::cerr << "[CartesianRobustInvDynController] Parameter '"
              << field_name
              << "' has invalid near-zero quaternion.\n";
    return false;
  }

  q.normalize();
  output = q.toRotationMatrix();

  return true;
}

Eigen::Quaterniond CartesianRobustInvDynController::rotationToQuaternion(
  const Eigen::Matrix3d & R)
{
  Eigen::Quaterniond q(R);
  q.normalize();
  return q;
}

Eigen::Vector3d CartesianRobustInvDynController::computeOrientationError(
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

void CartesianRobustInvDynController::fillPoseStamped(
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
  smm_controllers::CartesianRobustInvDynController,
  controller_interface::ControllerInterface)