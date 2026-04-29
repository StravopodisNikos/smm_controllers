#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "smm_controllers/core/smm_dynamics_adapter.hpp"

namespace smm_controllers
{

class PDGravityController : public controller_interface::ControllerInterface
{
public:
  PDGravityController();

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

  bool hold_initial_position_{true};

  std::string dynamics_data_dir_;
  std::string gravity_representation_{"body"};

  SmmDynamicsAdapter dynamics_adapter_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd gravity_;
};

}  // namespace smm_controllers