#include "smm_controllers/joint_space/joint_pd_effort_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

JointPDEffortController::JointPDEffortController()
: controller_interface::ControllerInterface(),
  hold_initial_position_(true)
{
}

controller_interface::CallbackReturn JointPDEffortController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<double>>("kp", std::vector<double>{});
    auto_declare<std::vector<double>>("kd", std::vector<double>{});
    auto_declare<std::vector<double>>("q_des", std::vector<double>{});
    auto_declare<bool>("hold_initial_position", true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointPDEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = make_interface_names(joint_names_, hardware_interface::HW_IF_EFFORT);
  return config;
}

controller_interface::InterfaceConfiguration
JointPDEffortController::state_interface_configuration() const
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

controller_interface::CallbackReturn JointPDEffortController::on_configure(
  const rclcpp_lifecycle::State &)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  kp_ = get_node()->get_parameter("kp").as_double_array();
  kd_ = get_node()->get_parameter("kd").as_double_array();
  q_des_ = get_node()->get_parameter("q_des").as_double_array();
  hold_initial_position_ = get_node()->get_parameter("hold_initial_position").as_bool();

  const auto n = joint_names_.size();

  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (kp_.size() != n || kd_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameters 'kp' and 'kd' must have the same size as 'joints'. joints=%zu kp=%zu kd=%zu",
      n, kp_.size(), kd_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!q_des_.empty() && q_des_.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'q_des' must be empty or have the same size as 'joints'. joints=%zu q_des=%zu",
      n, q_des_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  qd_des_.assign(n, 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "Configured JointPDEffortController with %zu joints.", n);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPDEffortController::on_activate(
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

    RCLCPP_INFO(get_node()->get_logger(), "Holding initial joint position.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPDEffortController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const auto n = joint_names_.size();

  for (size_t i = 0; i < n; ++i) {
    const auto q_opt = state_interfaces_[i].get_optional();
    const auto qd_opt = state_interfaces_[n + i].get_optional();

    if (!q_opt.has_value() || !qd_opt.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Failed to read state interfaces for joint '%s'.",
        joint_names_[i].c_str());

      return controller_interface::return_type::ERROR;
    }

    const double q = q_opt.value();
    const double qd = qd_opt.value();

    const double e = q_des_[i] - q;
    const double edot = qd_des_[i] - qd;

    const double tau = kp_[i] * e + kd_[i] * edot;

    const bool command_written = command_interfaces_[i].set_value(tau);

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

}  // namespace smm_controllers

PLUGINLIB_EXPORT_CLASS(
  smm_controllers::JointPDEffortController,
  controller_interface::ControllerInterface)