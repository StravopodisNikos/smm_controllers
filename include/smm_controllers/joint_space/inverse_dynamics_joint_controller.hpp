#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <memory>

#include <realtime_tools/realtime_buffer.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "smm_controllers/core/smm_dynamics_adapter.hpp"
#include "smm_controllers/core/trajectory_operation_utils.hpp"

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
  // ---------------------------------------------------------------------------
  // Reference command modes
  // ---------------------------------------------------------------------------    
  enum class ReferenceMode
  {
    HOLD,
    SINGLE_POINT,
    TRAJECTORY
  };

  ReferenceMode reference_mode_{ReferenceMode::HOLD};

  struct JointReferenceCommand
  {
    trajectory_msgs::msg::JointTrajectory trajectory;
    rclcpp::Time receive_time;
  };

  // ---------------------------------------------------------------------------
  // ROS INTERFACES
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_state_pub_;
  sensor_msgs::msg::JointState error_state_msg_;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> q_error_pubs_;
  std::vector<std_msgs::msg::Float64> q_error_msgs_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_state_pub_;
  sensor_msgs::msg::JointState desired_state_msg_;

  bool publish_desired_state_{true};

  // ---------------------------------------------------------------------------
  // Realtime reference command handling
  // ---------------------------------------------------------------------------
  realtime_tools::RealtimeBuffer<std::shared_ptr<JointReferenceCommand>> reference_buffer_;
  
  std::shared_ptr<JointReferenceCommand> active_command_;
  std::shared_ptr<JointReferenceCommand> last_processed_command_;

  // Using realtime buffer for realtime control - professional standards
  // ROS subscription callback:
  // >>> non-real-time side <<<
  // receives JointTrajectory
  // validates basic structure
  // writes command into realtime buffer

  // update():
  // >>> real-time side <<<
  // reads latest command from buffer
  // updates q_des/qdot_des/qddot_des

  // ---------------------------------------------------------------------------
  // Trajectory execution state
  // ---------------------------------------------------------------------------
  // Fixing time warnings. For robust trajectory execution, not rclcpp::Time is used!
  // We use the the controller update period
  double trajectory_elapsed_sec_{0.0};

  rclcpp::Time trajectory_start_time_; // USED WHEN: reference_mode_ == ReferenceMode::TRAJECTORY
  bool trajectory_running_{false};

  // Interpolation mode
  std::string trajectory_interpolation_mode_str_{"sync_cubic"};
  TrajectoryInterpolationMode trajectory_interpolation_mode_{
    TrajectoryInterpolationMode::SYNC_CUBIC};

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------    
  std::vector<std::string> joint_names_;

  std::vector<double> kp_;
  std::vector<double> kd_;

  std::vector<double> q_des_;
  std::vector<double> qdot_des_;
  std::vector<double> qddot_des_;

  bool hold_initial_position_{true};
  bool publish_error_state_{true};

  std::string dynamics_data_dir_;
  std::string dynamics_representation_{"body"};

  // ---------------------------------------------------------------------------
  // Dynamics aadapter
  // ---------------------------------------------------------------------------  
  SmmDynamicsAdapter dynamics_adapter_;

  // ---------------------------------------------------------------------------
  // Controller state vectors
  // ---------------------------------------------------------------------------      
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

  // ---------------------------------------------------------------------------
  // Reference handling
  // ---------------------------------------------------------------------------  
  void referenceCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  void processReferenceCommandIfAvailable(const rclcpp::Duration & period); // called inside update(), checks if a new command arrived

  bool acceptSinglePointCommand(
    const trajectory_msgs::msg::JointTrajectory & trajectory); // for backward cmpatibility with single point cmds

  bool startTrajectoryCommand(
    const trajectory_msgs::msg::JointTrajectory & trajectory); // stores trajectory and startd timer

  bool sampleActiveTrajectory(double elapsed_sec); // computes q_des/qdot_des/qddot_des during update()

  bool validateTrajectoryMessage(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const;

  // ---------------------------------------------------------------------------
  // Update-loop helper functions
  // ---------------------------------------------------------------------------
  bool readStateInterfaces();

  bool computeInverseDynamicsCommand();

  void publishDesiredState();

  void publishDebugState();

  bool writeCommandInterfaces();
};

}  // namespace smm_controllers