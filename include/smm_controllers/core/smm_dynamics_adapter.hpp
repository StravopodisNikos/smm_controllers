#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "smm_screws/core/RobotContextNdof.h"
#include "smm_screws/core/ScrewsDynamicsNdof.h"

namespace smm_controllers
{

class SmmDynamicsAdapter
{
public:
  SmmDynamicsAdapter() = default;

  bool initialize(
    const std::string & yaml_base_dir,
    const std::string & gravity_representation,
    const std::string & operational_dynamics_method = "exact_with_damped_fallback",
    double operational_damping = 1.0e-3);

  int dof() const;

  bool computeGravity(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qdot,
    Eigen::VectorXd & gravity);

  bool computeJointDynamics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qdot,
    Eigen::MatrixXd & mass_matrix,
    Eigen::MatrixXd & coriolis_matrix,
    Eigen::VectorXd & gravity);  

  bool computeNonredundantOperationalDynamics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qdot,
    Eigen::MatrixXd & operational_mass_matrix,
    Eigen::VectorXd & operational_coriolis_vector,
    Eigen::VectorXd & operational_gravity_vector,
    Eigen::MatrixXd & square_operational_jacobian,
    Eigen::MatrixXd & square_operational_jacobian_dot);

private:
  std::unique_ptr<RobotContextNdof> robot_context_ndof_;

  int dof_{0};

  ScrewsDynamicsNdof::DynamicsRepresentation gravity_representation_{
    ScrewsDynamicsNdof::DynamicsRepresentation::BODY
  };

  ScrewsDynamicsNdof::OperationalDynamicsMethod operational_dynamics_method_{
    ScrewsDynamicsNdof::OperationalDynamicsMethod::EXACT_WITH_DAMPED_FALLBACK
  };

  float operational_damping_{1.0e-3};

  std::vector<float> q_float_;
  std::vector<float> qdot_float_;
  std::vector<float> qddot_zero_;

  std::string dynamics_representation_str_{"body"};
  std::string body_frame_selection_str_{"joint"}; 

  static ScrewsDynamicsNdof::DynamicsRepresentation parseRepresentation(
    const std::string & representation);

  static ScrewsDynamicsNdof::OperationalDynamicsMethod parseOperationalDynamicsMethod(
    const std::string & method
  );  

  static std::unique_ptr<RobotAbstractBaseNdof> createRobotFromYaml(
    const std::string & yaml_base_dir);

  static int countPseudojointsFromAssembly(
    const std::string & yaml_base_dir);
};

}  // namespace smm_controllers