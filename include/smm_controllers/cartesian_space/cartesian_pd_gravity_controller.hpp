#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "smm_controllers/core/smm_dynamics_adapter.hpp"
#include "smm_controllers/core/smm_kinematics_adapter.hpp"

namespace smm_controllers
{

class CartesianPDGravityController : public controller_interface::ControllerInterface
{
public:
  CartesianPDGravityController();

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

  std::vector<double> kp_cartesian_;
  std::vector<double> kd_cartesian_;

  std::vector<double> x_des_;
  std::vector<double> xdot_des_;

  bool hold_initial_position_{true};
  bool publish_error_state_{true};
  bool publish_desired_state_{true};
  bool publish_current_state_{true};

  std::string kinematics_data_dir_;
  std::string dynamics_data_dir_;
  std::string gravity_representation_{"body"};
  std::string fixed_frame_{"world"};

  // ---------------------------------------------------------------------------
  // ROS interfaces
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_cartesian_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_cartesian_pub_;
  geometry_msgs::msg::PoseStamped desired_cartesian_msg_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_cartesian_pub_;
  geometry_msgs::msg::PoseStamped current_cartesian_msg_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr cartesian_error_pub_;
  geometry_msgs::msg::Vector3Stamped cartesian_error_msg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_state_pub_;
  sensor_msgs::msg::JointState error_state_msg_;

  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> q_error_pubs_;
  std::vector<std_msgs::msg::Float64> q_error_msgs_;

  // ---------------------------------------------------------------------------
  // Adapters
  // ---------------------------------------------------------------------------
  SmmKinematicsAdapter kinematics_adapter_;
  SmmDynamicsAdapter dynamics_adapter_;

  // ---------------------------------------------------------------------------
  // Joint-space vectors
  // ---------------------------------------------------------------------------
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;

  Eigen::VectorXd gravity_;
  Eigen::VectorXd tau_;

  // For joint debug only. Cartesian controller has no q_des.
  Eigen::VectorXd q_error_debug_;
  Eigen::VectorXd qdot_error_debug_;

  // ---------------------------------------------------------------------------
  // Cartesian vectors/matrices
  // ---------------------------------------------------------------------------
  Eigen::Vector3d x_;
  Eigen::Vector3d xdot_;

  Eigen::Vector3d x_des_eig_;
  Eigen::Vector3d xdot_des_eig_;

  Eigen::Vector3d x_error_;
  Eigen::Vector3d xdot_error_;

  Eigen::Vector3d f_cmd_;

  Eigen::MatrixXd Jv_;

  // ---------------------------------------------------------------------------
  // Reference handling
  // ---------------------------------------------------------------------------
  void desiredCartesianCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  bool acceptDesiredCartesianState(
    const geometry_msgs::msg::PoseStamped & msg);

  // ---------------------------------------------------------------------------
  // Update-loop helper functions
  // ---------------------------------------------------------------------------
  bool readStateInterfaces();

  bool computeCartesianKinematics();

  bool computeGravityVector();

  bool computeCartesianPDGravityCommand();

  void publishDesiredCartesianState();

  void publishCurrentCartesianState();

  void publishCartesianErrorState();

  void publishDebugState();

  bool writeCommandInterfaces();

  // ---------------------------------------------------------------------------
  // Utility helpers
  // ---------------------------------------------------------------------------
  static bool expandCartesianVector(
    const std::vector<double> & input,
    Eigen::Vector3d & output,
    const std::string & field_name);

  static void fillPoseStampedPositionOnly(
    geometry_msgs::msg::PoseStamped & msg,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & position);
};

}  // namespace smm_controllers