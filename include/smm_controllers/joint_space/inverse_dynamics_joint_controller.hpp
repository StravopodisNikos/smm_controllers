#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "smm_controllers/core/smm_dynamics_adapter.hpp"

namespace smm_controllers
{

class InverseDynamicsJointController : public controller_interface::ControllerInterface
{
public:
  InverseDynamicsJointController();

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
  std::vector<double> qdot_des_;
  std::vector<double> qddot_des_;

  bool hold_initial_position_{true};

  std::string dynamics_data_dir_;
  std::string dynamics_representation_{"body"};

  SmmDynamicsAdapter dynamics_adapter_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;
  Eigen::VectorXd q_des_eig_;
  Eigen::VectorXd qdot_des_eig_;
  Eigen::VectorXd qddot_des_eig_;

  Eigen::VectorXd e_;
  Eigen::VectorXd edot_;
  Eigen::VectorXd v_;
  Eigen::VectorXd gravity_;
  Eigen::VectorXd coriolis_vector_;
  Eigen::VectorXd tau_;

  Eigen::MatrixXd mass_matrix_;
  Eigen::MatrixXd coriolis_matrix_;

  // Publisher of error state
  bool publish_error_state_{true};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_state_pub_;
  sensor_msgs::msg::JointState error_state_msg_;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> q_error_pubs_;
  std::vector<std_msgs::msg::Float64> q_error_msgs_;

  // Subscriber for joint cmds
  void referenceCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_sub_;

};

}  // namespace smm_controllers