#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "smm_controllers/core/smm_dynamics_adapter.hpp"
#include "smm_controllers/core/smm_kinematics_adapter.hpp"

namespace smm_controllers
{

/**
 * @brief Robust-adaptive operational-space inverse dynamics controller for SMMs.
 *
 * This controller extends the already implemented robust inverse dynamics
 * controller by adding an adaptive operational-wrench compensation term.
 *
 * Operational-space model:
 *
 *   Mx(q) x_ddot + Cx(q, qdot) + Gx(q) + Delta_x = F
 *
 * Control structure:
 *
 *   e      = x_des - x
 *   e_dot  = xdot_des - xdot
 *   s      = e_dot + Lambda * e
 *
 *   F_nom  = Mx * a_ref + Cx + Gx
 *   F_rob  = K1 * s + K2 * tanh(kappa * s)
 *   F_ad   = rho_hat * tanh(kappa * s)
 *
 *   F      = F_nom + F_rob + F_ad
 *   tau    = Jx^T * F
 *
 * Adaptive uncertainty-bound update:
 *
 *   rho_hat_dot_i =
 *     gamma_i * max(|s_i| - deadzone, 0)
 *     - leakage_i * rho_hat_i
 *
 * where rho_hat_i estimates the unknown residual operational disturbance
 * bound in task component i.
 *
 * Notes:
 * - For a 3-DoF SMM, the task dimension is normally [x, y, z].
 * - For a 6-DoF SMM, the task dimension is [x, y, z, rx, ry, rz].
 * - Orientation terms can be disabled by setting the orientation gains,
 *   robust gains, and adaptive gains to zero.
 */
class CartesianRobustAdaptiveInvDynController
: public controller_interface::ControllerInterface
{
public:
  CartesianRobustAdaptiveInvDynController();

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type
  update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ==========================================================================
  // 1. Robot and controller parameters
  // ==========================================================================

  /// Active joint names, ordered exactly as the generated SMM active chain.
  std::vector<std::string> joint_names_;

  /// Task component names used for operational-space debug messages.
  /// 3-DoF: x, y, z
  /// 6-DoF: x, y, z, rx, ry, rz
  std::vector<std::string> task_component_names_;

  /// Position feedback gains used in the operational acceleration reference.
  std::vector<double> kp_position_;
  std::vector<double> kd_position_;

  /// Orientation feedback gains. Set to zero when orientation should be ignored.
  std::vector<double> kp_orientation_;
  std::vector<double> kd_orientation_;

  /// Optional desired Cartesian position loaded from YAML.
  std::vector<double> x_des_;

  /// Optional desired orientation quaternion [x, y, z, w] loaded from YAML.
  std::vector<double> orientation_des_;

  /// If true, the initial TCP pose is used as the desired pose until a command arrives.
  bool hold_initial_position_{true};

  /// Debug publishing switches.
  bool publish_error_state_{true};
  bool publish_desired_state_{true};
  bool publish_current_state_{true};
  bool publish_full_debug_state_{true};

  /// Paths to generated SMM synthesis data used by the kinematics/dynamics adapters.
  std::string kinematics_data_dir_;
  std::string dynamics_data_dir_;

  /// Dynamics representation used by SmmDynamicsAdapter: usually "body" or "spatial".
  std::string gravity_representation_{"body"};

  /// Frame id used by Cartesian pose/debug messages.
  std::string fixed_frame_{"world"};

  /// Operational dynamics method:
  /// "exact", "damped", or "exact_with_damped_fallback".
  std::string operational_dynamics_method_{"exact_with_damped_fallback"};

  /// Damping used when damped operational dynamics are selected or used as fallback.
  double operational_damping_{1.0e-3};

  // ==========================================================================
  // 2. Effort and velocity safety limits
  // ==========================================================================

  /// Scalar fallback effort limit if joint_effort_limits_ is not configured.
  double effort_limit_{80.0};

  /// Per-joint effort limits, ordered according to joint_names_.
  Eigen::VectorXd joint_effort_limits_;

  /// Enables controller-side velocity-limit torque filtering.
  bool enforce_velocity_limits_{true};

  /// Default velocity limit used when no per-joint limit is provided.
  double default_velocity_limit_{4.0841};

  /// Soft margin below the velocity limit where braking/reduction starts.
  double velocity_soft_margin_{0.25};

  /// Gain used to add braking torque near velocity limits.
  double velocity_brake_gain_{15.0};

  /// Per-joint velocity limits, ordered according to joint_names_.
  Eigen::VectorXd joint_velocity_limits_;

  /**
   * @brief Reduces torque that pushes a joint further into its velocity limit.
   *
   * This safety layer is applied after the operational wrench has been mapped
   * to joint torque, but before the final effort clamp.
   */
  void applyVelocityLimitTorqueFilter(
    const Eigen::VectorXd & qdot,
    Eigen::VectorXd & tau);

  // ==========================================================================
  // 3. ROS command input and state/debug publishers
  // ==========================================================================

  /// Desired Cartesian pose command.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    desired_cartesian_sub_;

  /// Desired Cartesian state publisher.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    desired_cartesian_pub_;
  geometry_msgs::msg::PoseStamped desired_cartesian_msg_;

  /// Current Cartesian state publisher.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    current_cartesian_pub_;
  geometry_msgs::msg::PoseStamped current_cartesian_msg_;

  /// Translational position error publisher.
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    position_error_pub_;
  geometry_msgs::msg::Vector3Stamped position_error_msg_;

  /// Orientation error publisher.
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    orientation_error_pub_;
  geometry_msgs::msg::Vector3Stamped orientation_error_msg_;

  /// Combined Cartesian error/debug state publisher.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    error_state_pub_;
  sensor_msgs::msg::JointState error_state_msg_;

  /// Per-joint error publishers for lightweight plotting.
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
    q_error_pubs_;
  std::vector<std_msgs::msg::Float64> q_error_msgs_;

  // ==========================================================================
  // 4. Full controller debug publishers
  // ==========================================================================

  /// Operational force command [Fx, Fy, Fz].
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    task_force_pub_;
  geometry_msgs::msg::Vector3Stamped task_force_msg_;

  /// Operational moment command [Mx, My, Mz].
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    task_moment_pub_;
  geometry_msgs::msg::Vector3Stamped task_moment_msg_;

  /// Task-space mapped torque contribution.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    task_torque_pub_;
  sensor_msgs::msg::JointState task_torque_msg_;

  /// Gravity torque/debug contribution.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    gravity_torque_pub_;
  sensor_msgs::msg::JointState gravity_torque_msg_;

  /// Final commanded joint torque after safety filters and effort clamps.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    commanded_torque_pub_;
  sensor_msgs::msg::JointState commanded_torque_msg_;

  /// Jacobian condition number.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    jacobian_condition_pub_;
  std_msgs::msg::Float64 jacobian_condition_msg_;

  /// Singular values of the square task Jacobian.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    jacobian_singular_values_pub_;
  sensor_msgs::msg::JointState jacobian_singular_values_msg_;

  /// Column norms of the operational Jacobian.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    jacobian_column_norms_pub_;
  sensor_msgs::msg::JointState jacobian_column_norms_msg_;

  // ==========================================================================
  // 5. Robust-adaptive debug publishers
  // ==========================================================================

  /// Sliding variable s = e_dot + Lambda * e.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    sliding_variable_pub_;
  sensor_msgs::msg::JointState sliding_variable_msg_;

  /// Nominal inverse-dynamics wrench F_nom = Mx*a_ref + Cx + Gx.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    nominal_wrench_pub_;
  sensor_msgs::msg::JointState nominal_wrench_msg_;

  /// Fixed robust wrench F_rob = K1*s + K2*tanh(kappa*s).
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    robust_wrench_pub_;
  sensor_msgs::msg::JointState robust_wrench_msg_;

  /// Adaptive wrench F_ad = rho_hat*tanh(kappa*s).
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    adaptive_wrench_pub_;
  sensor_msgs::msg::JointState adaptive_wrench_msg_;

  /// Adaptive uncertainty-bound estimate rho_hat.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    adaptive_rho_pub_;
  sensor_msgs::msg::JointState adaptive_rho_msg_;

  // ==========================================================================
  // 6. Kinematics and dynamics adapters
  // ==========================================================================

  SmmKinematicsAdapter kinematics_adapter_;
  SmmDynamicsAdapter dynamics_adapter_;

  // ==========================================================================
  // 7. Joint-space state and torque vectors
  // ==========================================================================

  /// Current active joint positions.
  Eigen::VectorXd q_;

  /// Current active joint velocities.
  Eigen::VectorXd qdot_;

  /// Joint-space gravity torque/vector from dynamics adapter.
  Eigen::VectorXd gravity_;

  /// Task-space torque contribution before final safety processing.
  Eigen::VectorXd tau_task_;

  /// Final commanded joint torque.
  Eigen::VectorXd tau_;

  /// Joint-space debug errors, when available.
  Eigen::VectorXd q_error_debug_;
  Eigen::VectorXd qdot_error_debug_;

  // ==========================================================================
  // 8. Cartesian pose state
  // ==========================================================================

  /// Current TCP position and translational velocity.
  Eigen::Vector3d x_;
  Eigen::Vector3d xdot_;

  /// Current TCP orientation and angular velocity.
  Eigen::Matrix3d R_;
  Eigen::Vector3d omega_;

  /// Desired TCP position, velocity, orientation, and angular velocity.
  Eigen::Vector3d x_des_eig_;
  Eigen::Vector3d xdot_des_eig_;
  Eigen::Matrix3d R_des_;
  Eigen::Vector3d omega_des_;

  /// Cartesian tracking errors using convention desired - current.
  Eigen::Vector3d x_error_;
  Eigen::Vector3d xdot_error_;
  Eigen::Vector3d orientation_error_;
  Eigen::Vector3d omega_error_;

  /// Split operational command for debug visualization.
  Eigen::Vector3d force_cmd_;
  Eigen::Vector3d moment_cmd_;

  /// Full operational wrench command. Dynamic-sized for 3-DoF/6-DoF support.
  Eigen::VectorXd wrench_cmd_;

  // ==========================================================================
  // 9. Jacobian and conditioning diagnostics
  // ==========================================================================

  /// Full 6 x n TCP operational Jacobian.
  Eigen::MatrixXd Jop_;

  /// Singular values of Jx.
  Eigen::VectorXd singular_values_;

  /// Column norms of Jop or Jx, depending on debug implementation.
  Eigen::VectorXd jacobian_column_norms_;

  /// Condition number of Jx.
  double jacobian_condition_{0.0};

  // ==========================================================================
  // 10. Operational-space dynamics
  // ==========================================================================

  /// Operational inertia matrix Mx.
  Eigen::MatrixXd Mx_;

  /// Operational Coriolis/centrifugal vector Cx.
  Eigen::VectorXd Cx_;

  /// Operational gravity vector Gx.
  Eigen::VectorXd Gx_;

  /// Square task Jacobian Jx used for nonredundant operational dynamics.
  Eigen::MatrixXd Jx_;

  /// Time derivative of the square task Jacobian.
  Eigen::MatrixXd dJx_;

  /// Operational position/orientation error vector.
  Eigen::VectorXd operational_error_;

  /// Operational velocity error vector.
  Eigen::VectorXd operational_velocity_error_;

  /// Reference operational acceleration a_ref.
  Eigen::VectorXd operational_acceleration_ref_;

  /// Total commanded operational wrench F.
  Eigen::VectorXd operational_wrench_;

  /// Scaling applied to the operational acceleration reference.
  double task_acceleration_scale_{1.0};

  /// Conditioning thresholds used to soften orientation or protect singular cases.
  double condition_soft_limit_{500.0};
  double condition_hard_limit_{2000.0};

  /// If true, orientation gains may be scaled when the task Jacobian is ill-conditioned.
  bool orientation_condition_scaling_{true};

  // ==========================================================================
  // 11. Fixed robust RIDOSC terms
  // ==========================================================================

  /// Sliding gain Lambda used in s = e_dot + Lambda*e.
  Eigen::VectorXd lambda_;

  /// Linear robust gain K1.
  Eigen::VectorXd k1_;

  /// Smooth switching robust gain K2.
  Eigen::VectorXd k2_;

  /// Sliding variable s.
  Eigen::VectorXd sliding_variable_;

  /// Nominal inverse-dynamics wrench F_nom.
  Eigen::VectorXd nominal_wrench_;

  /// Fixed robust wrench F_rob.
  Eigen::VectorXd robust_wrench_;

  /// Smoothness factor for tanh(kappa*s).
  double tanh_kappa_{5.0};

  // ==========================================================================
  // 12. Adaptive robust uncertainty-bound terms
  // ==========================================================================

  /// Last controller update period in seconds, used by the adaptive law.
  double update_period_sec_{0.0};

  /// Enables/disables the adaptive compensation term.
  bool adaptive_enabled_{true};

  /// Deadzone epsilon: adaptation is inactive for |s_i| <= epsilon.
  double adaptive_deadzone_{0.001};

  /// Adaptation gains gamma_i.
  /// Larger values make rho_hat grow faster when sliding error persists.
  Eigen::VectorXd adaptive_gain_;

  /// Leakage gains sigma_i.
  /// Leakage prevents parameter drift and slowly forgets unnecessary compensation.
  Eigen::VectorXd adaptive_leakage_;

  /// Adaptive uncertainty-bound estimate rho_hat_i.
  /// Each component estimates the residual operational disturbance bound.
  Eigen::VectorXd adaptive_rho_;

  /// Lower projection bound for rho_hat.
  Eigen::VectorXd adaptive_rho_min_;

  /// Upper projection bound for rho_hat.
  Eigen::VectorXd adaptive_rho_max_;

  /// Adaptive operational wrench F_ad = rho_hat*tanh(kappa*s).
  Eigen::VectorXd adaptive_wrench_;

  /**
   * @brief Updates the adaptive operational-wrench compensation.
   *
   * Implements:
   *
   *   rho_hat_dot_i =
   *     gamma_i * max(|s_i| - deadzone, 0)
   *     - leakage_i * rho_hat_i
   *
   *   F_ad_i =
   *     rho_hat_i * tanh(kappa * s_i)
   *
   * The update is performed in operational space before mapping the final wrench
   * to joint torque through tau = Jx^T * F.
   */
  void updateAdaptiveWrench(double dt);

  // ==========================================================================
  // 13. Reference handling
  // ==========================================================================

  void desiredCartesianCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  bool acceptDesiredCartesianState(
    const geometry_msgs::msg::PoseStamped & msg);

  // ==========================================================================
  // 14. Update-loop helper functions
  // ==========================================================================

  bool readStateInterfaces();

  bool computeCartesianPoseKinematics();

  bool computeSquareJacobianConditioning();

  bool computeCartesianInvDynCommand();

  void computeJacobianConditioning();

  void computeJacobianColumnNorms();

  bool writeCommandInterfaces();

  // ==========================================================================
  // 15. Publishing helper functions
  // ==========================================================================

  void publishDesiredCartesianState();

  void publishCurrentCartesianState();

  void publishCartesianErrorState();

  void publishDebugState();

  void publishFullDebugState();

  /// Publishes robust/adaptive internal controller vectors.
  void publishRobustAdaptiveDebugState();

  // ==========================================================================
  // 16. Utility helpers
  // ==========================================================================

  static bool expandVector3(
    const std::vector<double> & input,
    Eigen::Vector3d & output,
    const std::string & field_name);

  static bool quaternionVectorToRotation(
    const std::vector<double> & input,
    Eigen::Matrix3d & output,
    const std::string & field_name);

  static Eigen::Quaterniond rotationToQuaternion(
    const Eigen::Matrix3d & R);

  static Eigen::Vector3d computeOrientationError(
    const Eigen::Matrix3d & R_des,
    const Eigen::Matrix3d & R);

  static void fillPoseStamped(
    geometry_msgs::msg::PoseStamped & msg,
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & position,
    const Eigen::Matrix3d & orientation);
};

}  // namespace smm_controllers