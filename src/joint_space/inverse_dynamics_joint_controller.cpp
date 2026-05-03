#include "smm_controllers/joint_space/inverse_dynamics_joint_controller.hpp"

#include <stdexcept>
#include <algorithm>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "smm_controllers/core/smm_controller_utils.hpp"

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

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
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

  auto position_names = make_interface_names(joint_names_, hardware_interface::HW_IF_POSITION);
  auto velocity_names = make_interface_names(joint_names_, hardware_interface::HW_IF_VELOCITY);

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

    hold_initial_position_ = get_node()->get_parameter("hold_initial_position").as_bool();
    
    publish_error_state_ = get_node()->get_parameter("publish_error_state").as_bool();

    dynamics_data_dir_ = get_node()->get_parameter("dynamics_data_dir").as_string();
    dynamics_representation_ = get_node()->get_parameter("dynamics_representation").as_string();

    const auto n = joint_names_.size();

    if (n == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (kp_.size() != n || kd_.size() != n) {
        RCLCPP_ERROR(
        get_node()->get_logger(),
        "Parameters 'kp' and 'kd' must match joints size. joints=%zu kp=%zu kd=%zu",
        n, kp_.size(), kd_.size());
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
        n, dynamics_adapter_.dof());
        return controller_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < n; ++i) {
        qdot_des_eig_(static_cast<Eigen::Index>(i)) = qdot_des_[i];
        qddot_des_eig_(static_cast<Eigen::Index>(i)) = qddot_des_[i];
    }

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
      n, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != 2 * n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu.",
      2 * n, state_interfaces_.size());
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

    RCLCPP_INFO(get_node()->get_logger(), "InverseDynamicsJointController holding initial position.");
  }

  for (size_t i = 0; i < n; ++i) {
    q_des_eig_(static_cast<Eigen::Index>(i)) = q_des_[i];
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type InverseDynamicsJointController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
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

        return controller_interface::return_type::ERROR;
        }

        q_(static_cast<Eigen::Index>(i)) = q_opt.value();
        qdot_(static_cast<Eigen::Index>(i)) = qdot_opt.value();
    }

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

        return controller_interface::return_type::ERROR;
    }

    e_ = q_des_eig_ - q_;
    edot_ = qdot_des_eig_ - qdot_;

    for (size_t i = 0; i < n; ++i) {
        v_(static_cast<Eigen::Index>(i)) =
        qddot_des_eig_(static_cast<Eigen::Index>(i)) +
        kd_[i] * edot_(static_cast<Eigen::Index>(i)) +
        kp_[i] * e_(static_cast<Eigen::Index>(i));
    }

    coriolis_vector_ = coriolis_matrix_ * qdot_;

    tau_ = mass_matrix_ * v_ + coriolis_vector_ + gravity_;

    if (publish_error_state_ && error_state_pub_) {
    error_state_msg_.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    for (size_t i = 0; i < n; ++i) {
        const auto idx = static_cast<Eigen::Index>(i);

        error_state_msg_.position[i] = e_(idx);
        error_state_msg_.velocity[i] = edot_(idx);
        error_state_msg_.effort[i] = tau_(idx);
    }

    error_state_pub_->publish(error_state_msg_);

    for (size_t i = 0; i < n; ++i) {
        if (i < q_error_pubs_.size() && q_error_pubs_[i]) {
        q_error_msgs_[i].data = e_(static_cast<Eigen::Index>(i));
        q_error_pubs_[i]->publish(q_error_msgs_[i]);
        }
    }
    }

    for (size_t i = 0; i < n; ++i) {
        const bool command_written =
        command_interfaces_[i].set_value(tau_(static_cast<Eigen::Index>(i)));

        if (!command_written) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Failed to write effort command for joint '%s'.",
            joint_names_[i].c_str());

        return controller_interface::return_type::ERROR;
        }
    }

    return controller_interface::return_type::OK;
}

void InverseDynamicsJointController::referenceCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  const auto n = joint_names_.size();

  if (!msg) {
    return;
  }

  if (msg->points.empty()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received empty JointTrajectory reference. Ignoring command.");
    return;
  }

  const auto & point = msg->points.front();

  if (point.positions.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received JointTrajectory position size mismatch. Expected %zu, got %zu.",
      n,
      point.positions.size());
    return;
  }

  if (!point.velocities.empty() && point.velocities.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received JointTrajectory velocity size mismatch. Expected %zu, got %zu.",
      n,
      point.velocities.size());
    return;
  }

  if (!point.accelerations.empty() && point.accelerations.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received JointTrajectory acceleration size mismatch. Expected %zu, got %zu.",
      n,
      point.accelerations.size());
    return;
  }

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

  hold_initial_position_ = false;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Received new joint-space inverse dynamics reference with %zu joints.",
    n);
}

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::InverseDynamicsJointController,
  controller_interface::ControllerInterface)