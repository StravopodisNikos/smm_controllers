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

class CartesianRobustInvDynController : public controller_interface::ControllerInterface
{
    public:
        CartesianRobustInvDynController();

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

        std::vector<double> kp_position_;
        std::vector<double> kd_position_;

        std::vector<double> kp_orientation_;
        std::vector<double> kd_orientation_;

        std::vector<double> x_des_;
        std::vector<double> orientation_des_;

        bool hold_initial_position_{true};
        bool publish_error_state_{true};
        bool publish_desired_state_{true};
        bool publish_current_state_{true};
        bool publish_full_debug_state_{true};

        std::string kinematics_data_dir_;
        std::string dynamics_data_dir_;
        std::string gravity_representation_{"body"};
        std::string fixed_frame_{"world"};

        std::string operational_dynamics_method_{"exact_with_damped_fallback"};
        double operational_damping_{1.0e-3};
        
        // ---------------------------------------------------------------------------
        // Effort & Velocity Limits
        // ---------------------------------------------------------------------------
        double effort_limit_{80.0};
        Eigen::VectorXd joint_effort_limits_;
        bool enforce_velocity_limits_{true};
        double default_velocity_limit_{4.0841}; // same as xacro
        double velocity_soft_margin_{0.25};
        double velocity_brake_gain_{15.0};

        Eigen::VectorXd joint_velocity_limits_;

        void applyVelocityLimitTorqueFilter( const Eigen::VectorXd & qdot, Eigen::VectorXd & tau);

        // ---------------------------------------------------------------------------
        // ROS interfaces
        // ---------------------------------------------------------------------------
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_cartesian_sub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_cartesian_pub_;
        geometry_msgs::msg::PoseStamped desired_cartesian_msg_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_cartesian_pub_;
        geometry_msgs::msg::PoseStamped current_cartesian_msg_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_error_pub_;
        geometry_msgs::msg::Vector3Stamped position_error_msg_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr orientation_error_pub_;
        geometry_msgs::msg::Vector3Stamped orientation_error_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr error_state_pub_;
        sensor_msgs::msg::JointState error_state_msg_;

        std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> q_error_pubs_;
        std::vector<std_msgs::msg::Float64> q_error_msgs_;

        // Full debug publishers
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr task_force_pub_;
        geometry_msgs::msg::Vector3Stamped task_force_msg_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr task_moment_pub_;
        geometry_msgs::msg::Vector3Stamped task_moment_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr task_torque_pub_;
        sensor_msgs::msg::JointState task_torque_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_torque_pub_;
        sensor_msgs::msg::JointState gravity_torque_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commanded_torque_pub_;
        sensor_msgs::msg::JointState commanded_torque_msg_;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr jacobian_condition_pub_;
        std_msgs::msg::Float64 jacobian_condition_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jacobian_singular_values_pub_;
        sensor_msgs::msg::JointState jacobian_singular_values_msg_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jacobian_column_norms_pub_;
        sensor_msgs::msg::JointState jacobian_column_norms_msg_;

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
        Eigen::VectorXd tau_task_;
        Eigen::VectorXd tau_;

        Eigen::VectorXd q_error_debug_;
        Eigen::VectorXd qdot_error_debug_;

        // ---------------------------------------------------------------------------
        // Cartesian pose state
        // ---------------------------------------------------------------------------
        Eigen::Vector3d x_;
        Eigen::Vector3d xdot_;
        Eigen::Matrix3d R_;
        Eigen::Vector3d omega_;

        Eigen::Vector3d x_des_eig_;
        Eigen::Vector3d xdot_des_eig_;
        Eigen::Matrix3d R_des_;
        Eigen::Vector3d omega_des_;

        Eigen::Vector3d x_error_;
        Eigen::Vector3d xdot_error_;
        Eigen::Vector3d orientation_error_;
        Eigen::Vector3d omega_error_;

        Eigen::Vector3d force_cmd_;
        Eigen::Vector3d moment_cmd_;
        //Eigen::Matrix<double, 6, 1> wrench_cmd_;
        Eigen::VectorXd wrench_cmd_;

        Eigen::MatrixXd Jop_;

        Eigen::VectorXd singular_values_;
        Eigen::VectorXd jacobian_column_norms_;
        double jacobian_condition_{0.0};

        // ---------------------------------------------------------------------------
        // Operational space dynamics
        // ---------------------------------------------------------------------------
        Eigen::MatrixXd Mx_;
        Eigen::VectorXd Cx_;
        Eigen::VectorXd Gx_;
        Eigen::MatrixXd Jx_;
        Eigen::MatrixXd dJx_;

        Eigen::VectorXd operational_error_;
        Eigen::VectorXd operational_velocity_error_;
        Eigen::VectorXd operational_acceleration_ref_;
        Eigen::VectorXd operational_wrench_;

        double task_acceleration_scale_{1.0};
        double condition_soft_limit_{500.0};
        double condition_hard_limit_{2000.0};
        bool orientation_condition_scaling_{true};

        Eigen::VectorXd lambda_;
        Eigen::VectorXd k1_;
        Eigen::VectorXd k2_;
        Eigen::VectorXd sliding_variable_;
        Eigen::VectorXd nominal_wrench_;
        Eigen::VectorXd robust_wrench_;

        double tanh_kappa_{5.0};

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

        bool computeCartesianPoseKinematics();
        
        bool computeSquareJacobianConditioning();

        bool computeCartesianInvDynCommand();

        void computeJacobianConditioning();

        void computeJacobianColumnNorms();

        void publishDesiredCartesianState();

        void publishCurrentCartesianState();

        void publishCartesianErrorState();

        void publishDebugState();

        void publishFullDebugState();

        bool writeCommandInterfaces();

        // ---------------------------------------------------------------------------
        // Utility helpers
        // ---------------------------------------------------------------------------
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