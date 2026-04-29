#pragma once

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace smm_controllers
{

class JointPDEffortController : public controller_interface::ControllerInterface
{
public:
  JointPDEffortController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;

  std::vector<double> kp_;
  std::vector<double> kd_;

  std::vector<double> q_des_;
  std::vector<double> qd_des_;

  bool hold_initial_position_;
};

}  // namespace smm_controllers