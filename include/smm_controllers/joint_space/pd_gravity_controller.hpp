#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  std::vector<std::string> joint_names_;

  std::vector<double> kp_;
  std::vector<double> kd_;

  std::vector<double> q_des_;
  std::vector<double> qdot_des_;

  bool hold_initial_position_{true};
  bool publish_error_state_{true};

  std::string dynamics_data_dir_;
  std::string gravity_representation_{"body"};

  // ---------------------------------------------------------------------------
  // ROS interfaces
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_state_pub_;
  sensor_msgs::msg::JointState error_state_msg_;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> q_error_pubs_;
  std::vector<std_msgs::msg::Float64> q_error_msgs_;

  // ---------------------------------------------------------------------------
  // Dynamics adapter
  // ---------------------------------------------------------------------------
  SmmDynamicsAdapter dynamics_adapter_;

  // ---------------------------------------------------------------------------
  // Controller state vectors
  // ---------------------------------------------------------------------------
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;

  Eigen::VectorXd q_des_eig_;
  Eigen::VectorXd qdot_des_eig_;

  Eigen::VectorXd e_;
  Eigen::VectorXd edot_;

  Eigen::VectorXd gravity_;
  Eigen::VectorXd tau_;

  // ---------------------------------------------------------------------------
  // Reference handling
  // ---------------------------------------------------------------------------
  void referenceCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  bool validateTrajectoryMessage(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const;

  bool acceptSinglePointCommand(
    const trajectory_msgs::msg::JointTrajectory & trajectory);

  // ---------------------------------------------------------------------------
  // Update-loop helper functions
  // ---------------------------------------------------------------------------
  bool readStateInterfaces();

  bool computePDGravityCommand();

  void publishDebugState();

  bool writeCommandInterfaces();
};

}  // namespace smm_controllers