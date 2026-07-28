#include "smm_controllers/cartesian_space/cartesian_robust_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <mutex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

CartesianRobustImpedanceController::CartesianRobustImpedanceController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CartesianRobustImpedanceController::on_init()
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

    auto_declare<std::vector<double>>("x_des", std::vector<double>{});
    auto_declare<std::vector<double>>("orientation_des", std::vector<double>{});

    auto_declare<bool>("hold_initial_position", true);
    auto_declare<bool>("publish_error_state", true);
    auto_declare<bool>("publish_desired_state", true);
    auto_declare<bool>("publish_current_state", true);
    auto_declare<bool>("publish_full_debug_state", true);

    // Velocity & Torque limits
    auto_declare<double>("effort_limit", 80.0); // this is a compromise between stepper and dxl max torques
    auto_declare<bool>("enforce_velocity_limits", true);
    auto_declare<double>("default_velocity_limit", 4.0841); // same as xacro
    auto_declare<double>("velocity_soft_margin", 0.25);
    auto_declare<double>("velocity_brake_gain", 15.0);
    auto_declare<std::vector<double>>("joint_velocity_limits", std::vector<double>{});
    auto_declare<std::vector<double>>("joint_effort_limits", std::vector<double>{});

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

    // Cartesian Impedance Model
    auto_declare<std::vector<double>>(
    "impedance_mass_position",
    std::vector<double>{5.0, 5.0, 5.0});

    auto_declare<std::vector<double>>(
    "impedance_damping_position",
    std::vector<double>{70.0, 70.0, 90.0});

    auto_declare<std::vector<double>>(
    "impedance_stiffness_position",
    std::vector<double>{500.0, 500.0, 700.0});

    auto_declare<std::vector<double>>(
    "impedance_mass_orientation",
    std::vector<double>{1.0, 1.0, 1.0});

    auto_declare<std::vector<double>>(
    "impedance_damping_orientation",
    std::vector<double>{0.0, 0.0, 0.0});

    auto_declare<std::vector<double>>(
    "impedance_stiffness_orientation",
    std::vector<double>{0.0, 0.0, 0.0});  
    
    // External TCP interaction wrench.
    //
    // The controller expects W_ext at TCP, expressed in fixed_frame/world.
    // This is NOT raw force_sensor_meas_frame data.
    auto_declare<bool>("use_external_wrench", true);
    auto_declare<bool>("subtract_external_wrench_from_command", true);
    auto_declare<bool>("require_external_wrench_frame_match", true);

    auto_declare<double>("external_wrench_sign", 1.0);
    auto_declare<double>("external_wrench_timeout", 0.2);
    auto_declare<double>("external_wrench_filter_alpha", 0.2);

    auto_declare<double>("external_wrench_deadband_force", 0.2);
    auto_declare<double>("external_wrench_deadband_torque", 0.02);

    auto_declare<double>("external_wrench_limit_force", 80.0);
    auto_declare<double>("external_wrench_limit_torque", 10.0);

    // Desired interaction wrench at TCP, expressed in fixed_frame/world.
    auto_declare<std::vector<double>>(
      "desired_wrench_position",
      std::vector<double>{0.0, 0.0, 0.0});

    auto_declare<std::vector<double>>(
      "desired_wrench_orientation",
      std::vector<double>{0.0, 0.0, 0.0});


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
CartesianRobustImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
CartesianRobustImpedanceController::state_interface_configuration() const
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

controller_interface::CallbackReturn CartesianRobustImpedanceController::on_configure(
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
    
    impedance_mass_position_ =
        get_node()->get_parameter("impedance_mass_position").as_double_array();

    impedance_damping_position_ =
        get_node()->get_parameter("impedance_damping_position").as_double_array();

    impedance_stiffness_position_ =
        get_node()->get_parameter("impedance_stiffness_position").as_double_array();

    impedance_mass_orientation_ =
        get_node()->get_parameter("impedance_mass_orientation").as_double_array();

    impedance_damping_orientation_ =
        get_node()->get_parameter("impedance_damping_orientation").as_double_array();

    impedance_stiffness_orientation_ =
        get_node()->get_parameter("impedance_stiffness_orientation").as_double_array();

    use_external_wrench_ =
        get_node()->get_parameter("use_external_wrench").as_bool();

    subtract_external_wrench_from_command_ =
        get_node()->get_parameter("subtract_external_wrench_from_command").as_bool();

    require_external_wrench_frame_match_ =
        get_node()->get_parameter("require_external_wrench_frame_match").as_bool();

    external_wrench_sign_ =
        get_node()->get_parameter("external_wrench_sign").as_double();

    external_wrench_timeout_ =
        get_node()->get_parameter("external_wrench_timeout").as_double();

    external_wrench_filter_alpha_ =
        get_node()->get_parameter("external_wrench_filter_alpha").as_double();

    external_wrench_deadband_force_ =
        get_node()->get_parameter("external_wrench_deadband_force").as_double();

    external_wrench_deadband_torque_ =
        get_node()->get_parameter("external_wrench_deadband_torque").as_double();

    external_wrench_limit_force_ =
        get_node()->get_parameter("external_wrench_limit_force").as_double();

    external_wrench_limit_torque_ =
        get_node()->get_parameter("external_wrench_limit_torque").as_double();

    desired_wrench_position_ =
        get_node()->get_parameter("desired_wrench_position").as_double_array();

    desired_wrench_orientation_ =
        get_node()->get_parameter("desired_wrench_orientation").as_double_array();

    external_wrench_filter_alpha_ =
        std::clamp(external_wrench_filter_alpha_, 0.0, 1.0);

    if (external_wrench_timeout_ <= 0.0) {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "external_wrench_timeout must be positive. Resetting to 0.2 s.");
        external_wrench_timeout_ = 0.2;
    }

    external_wrench_deadband_force_ =
        std::max(0.0, external_wrench_deadband_force_);

    external_wrench_deadband_torque_ =
        std::max(0.0, external_wrench_deadband_torque_);

    external_wrench_limit_force_ =
        std::max(1.0e-9, external_wrench_limit_force_);

    external_wrench_limit_torque_ =
        std::max(1.0e-9, external_wrench_limit_torque_);
        
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

    // Effort limits
    const auto joint_effort_limits_param =
      get_node()->get_parameter("joint_effort_limits").as_double_array();

    joint_effort_limits_.resize(n);

    if (joint_effort_limits_param.empty()) {
      joint_effort_limits_.setConstant(effort_limit_);
    } else if (joint_effort_limits_param.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        joint_effort_limits_(i) = joint_effort_limits_param[i];
      }
    } else {
      throw std::runtime_error(
        "joint_effort_limits must be empty or have size equal to number of active joints.");
    }

    for (Eigen::Index i = 0; i < joint_effort_limits_.size(); ++i) {
      if (joint_effort_limits_(i) <= 0.0) {
        throw std::runtime_error("All joint_effort_limits values must be positive.");
      }
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Joint effort limits loaded. First joint limit=%.3f Nm, scalar fallback=%.3f Nm",
      joint_effort_limits_(0),
      effort_limit_);

    std::ostringstream oss;
    oss << "Joint effort limits: ";
    for (Eigen::Index i = 0; i < joint_effort_limits_.size(); ++i) {
      oss << joint_names_[static_cast<std::size_t>(i)]
          << "=" << joint_effort_limits_(i) << " ";
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "%s",
      oss.str().c_str());

    // Velocity limits
    enforce_velocity_limits_ =
      get_node()->get_parameter("enforce_velocity_limits").as_bool();

    default_velocity_limit_ =
      get_node()->get_parameter("default_velocity_limit").as_double();

    velocity_soft_margin_ =
      get_node()->get_parameter("velocity_soft_margin").as_double();

    velocity_brake_gain_ =
      get_node()->get_parameter("velocity_brake_gain").as_double();

    const auto joint_velocity_limits_param =
      get_node()->get_parameter("joint_velocity_limits").as_double_array();

    if (default_velocity_limit_ <= 0.0) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "default_velocity_limit must be positive. Resetting to 4.0841 rad/s.");
      default_velocity_limit_ = 4.0841;
    }

    if (velocity_soft_margin_ <= 0.0) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "velocity_soft_margin must be positive. Resetting to 0.25 rad/s.");
      velocity_soft_margin_ = 0.25;
    }

    if (velocity_brake_gain_ < 0.0) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "velocity_brake_gain must be non-negative. Resetting to 15.0.");
      velocity_brake_gain_ = 15.0;
    }

    joint_velocity_limits_.resize(n);

    if (joint_velocity_limits_param.empty()) {
      joint_velocity_limits_.setConstant(default_velocity_limit_);
    } else if (joint_velocity_limits_param.size() == 1) {
      joint_velocity_limits_.setConstant(joint_velocity_limits_param[0]);
    } else if (joint_velocity_limits_param.size() == n) {
      for (std::size_t i = 0; i < n; ++i) {
        joint_velocity_limits_(i) = joint_velocity_limits_param[i];
      }
    } else {
      throw std::runtime_error(
        "joint_velocity_limits must be empty, size 1, or equal to number of active joints.");
    }

    for (Eigen::Index i = 0; i < joint_velocity_limits_.size(); ++i) {
      if (joint_velocity_limits_(i) <= 0.0) {
        throw std::runtime_error("All joint_velocity_limits values must be positive.");
      }
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Velocity-limit torque filter: enabled=%s, default_velocity_limit=%.4f rad/s, "
      "soft_margin=%.4f rad/s, brake_gain=%.4f",
      enforce_velocity_limits_ ? "true" : "false",
      default_velocity_limit_,
      velocity_soft_margin_,
      velocity_brake_gain_);

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

    impedance_mass_.resize(static_cast<Eigen::Index>(n));
    impedance_damping_.resize(static_cast<Eigen::Index>(n));
    impedance_stiffness_.resize(static_cast<Eigen::Index>(n));
    impedance_acceleration_.resize(static_cast<Eigen::Index>(n));

    impedance_mass_.setZero();
    impedance_damping_.setZero();
    impedance_stiffness_.setZero();
    impedance_acceleration_.setZero();

    external_wrench_raw_.resize(static_cast<Eigen::Index>(n));
    external_wrench_filtered_.resize(static_cast<Eigen::Index>(n));
    external_wrench_used_.resize(static_cast<Eigen::Index>(n));

    desired_interaction_wrench_.resize(static_cast<Eigen::Index>(n));
    desired_interaction_wrench_used_.resize(static_cast<Eigen::Index>(n));

    external_wrench_raw_.setZero();
    external_wrench_filtered_.setZero();
    external_wrench_used_.setZero();

    desired_interaction_wrench_.setZero();
    desired_interaction_wrench_used_.setZero();

    if (!loadTaskVectorFromPositionOrientation(
        desired_wrench_position_,
        desired_wrench_orientation_,
        desired_interaction_wrench_,
        "desired_wrench"))
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    desired_interaction_wrench_used_ = desired_interaction_wrench_;

    external_wrench_received_ = false;
    last_external_wrench_time_ = get_node()->now();

    if (!loadTaskVectorFromPositionOrientation(
        impedance_mass_position_,
        impedance_mass_orientation_,
        impedance_mass_,
        "impedance_mass"))
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!loadTaskVectorFromPositionOrientation(
        impedance_damping_position_,
        impedance_damping_orientation_,
        impedance_damping_,
        "impedance_damping"))
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    if (!loadTaskVectorFromPositionOrientation(
        impedance_stiffness_position_,
        impedance_stiffness_orientation_,
        impedance_stiffness_,
        "impedance_stiffness"))
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    for (Eigen::Index i = 0; i < impedance_mass_.size(); ++i) {
        if (impedance_mass_(i) <= 1.0e-9) {
            RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid impedance mass at task index %ld. Value must be positive.",
            static_cast<long>(i));
            return controller_interface::CallbackReturn::ERROR;
        }

        if (impedance_damping_(i) < 0.0) {
            RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid impedance damping at task index %ld. Value must be non-negative.",
            static_cast<long>(i));
            return controller_interface::CallbackReturn::ERROR;
        }

        if (impedance_stiffness_(i) < 0.0) {
            RCLCPP_ERROR(
            get_node()->get_logger(),
            "Invalid impedance stiffness at task index %ld. Value must be non-negative.",
            static_cast<long>(i));
            return controller_interface::CallbackReturn::ERROR;
        }
    }

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
            &CartesianRobustImpedanceController::desiredCartesianCallback,
            this,
            std::placeholders::_1));

    external_wrench_sub_ =
        get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "~/external_wrench",
        rclcpp::SystemDefaultsQoS(),
        std::bind(
            &CartesianRobustImpedanceController::externalWrenchCallback,
            this,
            std::placeholders::_1));

    desired_interaction_wrench_sub_ =
        get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "~/desired_interaction_wrench",
        rclcpp::SystemDefaultsQoS(),
        std::bind(
            &CartesianRobustImpedanceController::desiredInteractionWrenchCallback,
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
        "Configured CartesianRobustImpedanceController with %zu joints.", n);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Cartesian pose reference topic: ~/desired_cartesian_state");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianRobustImpedanceController::on_activate(
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

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Command interface order:");

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "  command_interfaces_[%zu] = %s/%s ; expected joint_names_[%zu] = %s",
      i,
      command_interfaces_[i].get_prefix_name().c_str(),
      command_interfaces_[i].get_interface_name().c_str(),
      i,
      joint_names_[i].c_str());
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
      "CartesianRobustImpedanceController holding initial TCP pose.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianRobustImpedanceController::update(
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

void CartesianRobustImpedanceController::desiredCartesianCallback(
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

bool CartesianRobustImpedanceController::acceptDesiredCartesianState(
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

bool CartesianRobustImpedanceController::readStateInterfaces()
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

bool CartesianRobustImpedanceController::computeCartesianPoseKinematics()
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

bool CartesianRobustImpedanceController::computeSquareJacobianConditioning()
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

bool CartesianRobustImpedanceController::computeCartesianInvDynCommand()
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

  /*
   * Robust Impedance Controller, RIC.
   *
   * Current version:
   *   free-space impedance, no measured TCP interaction wrench yet.
   *
   * Controller convention:
   *   e     = desired - current
   *   e_dot = desired_velocity - current_velocity
   *
   * Desired task-space impedance acceleration:
   *
   *   a_imp =
   *     task_acceleration_scale *
   *     M_imp^{-1} (D_imp * e_dot + K_imp * e)
   *
   * Robust operational-space wrench:
   *
   *   s = e_dot + Lambda * e
   *
   *   W_nom =
   *     Mx * a_imp + Cx + Gx
   *
   *   W_rob =
   *     K1 * s + K2 * tanh(kappa * s)
   *
   *   W_total =
   *     W_nom + W_rob
   *
   *   tau =
   *     Jx^T * W_total
   */
  updateExternalWrenchForControl();

  impedance_acceleration_.setZero();

  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Index ei = static_cast<Eigen::Index>(i);

    const double m_i =
      std::max(impedance_mass_(ei), 1.0e-9);

    /*
     * Real-test-aligned robust interaction impedance.
     *
     * Convention:
     *   e     = desired - current
     *   e_dot = desired_velocity - current_velocity
     *
     *   W_ext = environment-on-robot wrench at TCP, expressed in fixed_frame/world
     *   W_des = desired interaction wrench at TCP, expressed in fixed_frame/world
     *
     *   a_imp =
     *     task_acceleration_scale *
     *     M_imp^{-1}
     *     (
     *       D_imp * e_dot
     *       + K_imp * e
     *       + W_ext
     *       - W_des
     *     )
     */
    impedance_acceleration_(ei) =
      task_acceleration_scale_ *
      (
        impedance_damping_(ei) * operational_velocity_error_(ei) +
        impedance_stiffness_(ei) * operational_error_(ei) +
        external_wrench_used_(ei) -
        desired_interaction_wrench_used_(ei)
      ) / m_i;
  }

  operational_acceleration_ref_ = impedance_acceleration_;

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

  sliding_variable_ =
    operational_velocity_error_ +
    lambda_.cwiseProduct(operational_error_);

  /*
   * Operational dynamics convention:
   *
   *   Mx*xdd + Cx + Gx = W_cmd + W_ext
   *
   * Therefore, when W_ext is a physically applied real/contact wrench:
   *
   *   W_cmd = Mx*a_imp + Cx + Gx - W_ext
   *
   * For our real-test-aligned implementation this should stay enabled.
   */
  nominal_wrench_ =
    Mx_ * operational_acceleration_ref_ +
    Cx_ +
    Gx_;

  if (subtract_external_wrench_from_command_) {
    nominal_wrench_ -= external_wrench_used_;
  }

  const Eigen::Index task_dim = static_cast<Eigen::Index>(n);

  robust_wrench_.resize(task_dim);
  robust_wrench_.setZero();

  for (Eigen::Index i = 0; i < task_dim; ++i) {
    const double s_i = sliding_variable_(i);

    robust_wrench_(i) =
      k1_(i) * s_i +
      k2_(i) * std::tanh(tanh_kappa_ * s_i);
  }
  
  operational_wrench_ =
    nominal_wrench_ +
    robust_wrench_;

  tau_ =
    Jx_.transpose() * operational_wrench_;

  // Velocity limiter.
  applyVelocityLimitTorqueFilter(qdot_, tau_);

  // Effort clamp.
  std::ostringstream clamp_oss;
  clamp_oss << "EFFORT CLAMP SUMMARY: ";

  for (Eigen::Index i = 0; i < tau_.size(); ++i) {
    if (!std::isfinite(tau_(i))) {
      tau_(i) = 0.0;
    }

    const double tau_before_clamp = tau_(i);

    const double limit_i =
      joint_effort_limits_.size() == tau_.size()
        ? joint_effort_limits_(i)
        : effort_limit_;

    tau_(i) = std::clamp(
      tau_(i),
      -limit_i,
      limit_i);

    clamp_oss
      << "[" << i << " "
      << joint_names_[static_cast<std::size_t>(i)]
      << " raw=" << tau_before_clamp
      << " clamped=" << tau_(i)
      << " limit=" << limit_i
      << "] ";
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "%s",
    clamp_oss.str().c_str());

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
   * This is not used separately in the commanded torque.
   */
  (void)dynamics_adapter_.computeGravity(q_, qdot_, gravity_);

  q_error_debug_.setZero();
  qdot_error_debug_.setZero();

  return true;
}

void CartesianRobustImpedanceController::externalWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (external_wrench_raw_.size() == 0) {
    return;
  }

  if (require_external_wrench_frame_match_ &&
      !msg->header.frame_id.empty() &&
      msg->header.frame_id != fixed_frame_)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Rejected external wrench because frame_id='%s' but fixed_frame='%s'. "
      "Controller expects W_ext_tcp_world.",
      msg->header.frame_id.c_str(),
      fixed_frame_.c_str());

    return;
  }

  Eigen::Matrix<double, 6, 1> wrench6;
  wrench6 << msg->wrench.force.x,
             msg->wrench.force.y,
             msg->wrench.force.z,
             msg->wrench.torque.x,
             msg->wrench.torque.y,
             msg->wrench.torque.z;

  std::lock_guard<std::mutex> lock(external_wrench_mutex_);

  external_wrench_raw_.setZero();

  const Eigen::Index count =
    std::min<Eigen::Index>(external_wrench_raw_.size(), 6);

  for (Eigen::Index i = 0; i < count; ++i) {
    external_wrench_raw_(i) =
      external_wrench_sign_ * wrench6(i);
  }

  applyExternalWrenchDeadbandAndClamp(external_wrench_raw_);

  if (!external_wrench_received_) {
    external_wrench_filtered_ = external_wrench_raw_;
    external_wrench_received_ = true;
  } else {
    external_wrench_filtered_ =
      external_wrench_filter_alpha_ * external_wrench_raw_ +
      (1.0 - external_wrench_filter_alpha_) * external_wrench_filtered_;
  }

  // Use receipt time. This avoids sim-time/system-time stamp mismatch.
  last_external_wrench_time_ = get_node()->now();
}

void CartesianRobustImpedanceController::desiredInteractionWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (desired_interaction_wrench_.size() == 0) {
    return;
  }

  if (require_external_wrench_frame_match_ &&
      !msg->header.frame_id.empty() &&
      msg->header.frame_id != fixed_frame_)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Rejected desired interaction wrench because frame_id='%s' but fixed_frame='%s'.",
      msg->header.frame_id.c_str(),
      fixed_frame_.c_str());

    return;
  }

  Eigen::Matrix<double, 6, 1> wrench6;
  wrench6 << msg->wrench.force.x,
             msg->wrench.force.y,
             msg->wrench.force.z,
             msg->wrench.torque.x,
             msg->wrench.torque.y,
             msg->wrench.torque.z;

  std::lock_guard<std::mutex> lock(external_wrench_mutex_);

  desired_interaction_wrench_.setZero();

  const Eigen::Index count =
    std::min<Eigen::Index>(desired_interaction_wrench_.size(), 6);

  for (Eigen::Index i = 0; i < count; ++i) {
    desired_interaction_wrench_(i) = wrench6(i);
  }

  applyExternalWrenchDeadbandAndClamp(desired_interaction_wrench_);
}

void CartesianRobustImpedanceController::updateExternalWrenchForControl()
{
  external_wrench_used_.setZero();

  std::lock_guard<std::mutex> lock(external_wrench_mutex_);

  desired_interaction_wrench_used_ = desired_interaction_wrench_;

  if (!use_external_wrench_) {
    return;
  }

  if (!external_wrench_received_) {
    return;
  }

  const double age =
    (get_node()->now() - last_external_wrench_time_).seconds();

  if (!std::isfinite(age) || age > external_wrench_timeout_) {
    external_wrench_used_.setZero();

    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "External wrench timeout. Using zero external wrench.");

    return;
  }

  external_wrench_used_ = external_wrench_filtered_;

  applyExternalWrenchDeadbandAndClamp(external_wrench_used_);
}

void CartesianRobustImpedanceController::applyExternalWrenchDeadbandAndClamp(
  Eigen::VectorXd & wrench) const
{
  for (Eigen::Index i = 0; i < wrench.size(); ++i) {
    const bool force_component = i < 3;

    const double deadband =
      force_component ?
      external_wrench_deadband_force_ :
      external_wrench_deadband_torque_;

    const double limit =
      force_component ?
      external_wrench_limit_force_ :
      external_wrench_limit_torque_;

    if (std::abs(wrench(i)) < deadband) {
      wrench(i) = 0.0;
      continue;
    }

    wrench(i) = std::clamp(wrench(i), -limit, limit);
  }
}

void CartesianRobustImpedanceController::computeJacobianConditioning()
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

void CartesianRobustImpedanceController::computeJacobianColumnNorms()
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

void CartesianRobustImpedanceController::publishDesiredCartesianState()
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

void CartesianRobustImpedanceController::publishCurrentCartesianState()
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

void CartesianRobustImpedanceController::publishCartesianErrorState()
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

void CartesianRobustImpedanceController::publishDebugState()
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

void CartesianRobustImpedanceController::publishFullDebugState()
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

bool CartesianRobustImpedanceController::writeCommandInterfaces()
{
  return write_vector_to_command_interfaces(
    command_interfaces_,
    joint_names_,
    tau_,
    get_node()->get_logger(),
    *get_node()->get_clock(),
    "effort");
}

bool CartesianRobustImpedanceController::expandVector3(
  const std::vector<double> & input,
  Eigen::Vector3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setZero();
    return true;
  }

  if (input.size() != 3) {
    std::cerr << "[CartesianRobustImpedanceController] Parameter '"
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

bool CartesianRobustImpedanceController::quaternionVectorToRotation(
  const std::vector<double> & input,
  Eigen::Matrix3d & output,
  const std::string & field_name)
{
  if (input.empty()) {
    output.setIdentity();
    return true;
  }

  if (input.size() != 4) {
    std::cerr << "[CartesianRobustImpedanceController] Parameter '"
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
    std::cerr << "[CartesianRobustImpedanceController] Parameter '"
              << field_name
              << "' has invalid near-zero quaternion.\n";
    return false;
  }

  q.normalize();
  output = q.toRotationMatrix();

  return true;
}

Eigen::Quaterniond CartesianRobustImpedanceController::rotationToQuaternion(
  const Eigen::Matrix3d & R)
{
  Eigen::Quaterniond q(R);
  q.normalize();
  return q;
}

Eigen::Vector3d CartesianRobustImpedanceController::computeOrientationError(
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

void CartesianRobustImpedanceController::fillPoseStamped(
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

void CartesianRobustImpedanceController::applyVelocityLimitTorqueFilter(
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

bool CartesianRobustImpedanceController::loadTaskVectorFromPositionOrientation(
  const std::vector<double> & position_values,
  const std::vector<double> & orientation_values,
  Eigen::VectorXd & output,
  const std::string & field_name) const
{
  const Eigen::Index n =
    static_cast<Eigen::Index>(joint_names_.size());

  if (output.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Internal size mismatch while loading '%s'. Expected %ld, got %ld.",
      field_name.c_str(),
      static_cast<long>(n),
      static_cast<long>(output.size()));
    return false;
  }

  if (position_values.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter '%s_position' must have exactly 3 values. Got %zu.",
      field_name.c_str(),
      position_values.size());
    return false;
  }

  if (orientation_values.size() != 3) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter '%s_orientation' must have exactly 3 values. Got %zu.",
      field_name.c_str(),
      orientation_values.size());
    return false;
  }

  output.setZero();

  if (n >= 1) {
    output(0) = position_values[0];
  }

  if (n >= 2) {
    output(1) = position_values[1];
  }

  if (n >= 3) {
    output(2) = position_values[2];
  }

  if (n >= 4) {
    output(3) = orientation_values[0];
  }

  if (n >= 5) {
    output(4) = orientation_values[1];
  }

  if (n >= 6) {
    output(5) = orientation_values[2];
  }

  return true;
}

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::CartesianRobustImpedanceController,
  controller_interface::ControllerInterface)