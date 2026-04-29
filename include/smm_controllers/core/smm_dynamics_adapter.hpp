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
    const std::string & gravity_representation);

  int dof() const;

  bool computeGravity(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qdot,
    Eigen::VectorXd & gravity);

private:
  std::unique_ptr<RobotContextNdof> robot_context_ndof_;

  int dof_{0};

  ScrewsDynamicsNdof::DynamicsRepresentation gravity_representation_{
    ScrewsDynamicsNdof::DynamicsRepresentation::BODY
  };

  std::vector<float> q_float_;
  std::vector<float> qdot_float_;
  std::vector<float> qddot_zero_;

  static ScrewsDynamicsNdof::DynamicsRepresentation parseRepresentation(
    const std::string & representation);

  static std::unique_ptr<RobotAbstractBaseNdof> createRobotFromYaml(
    const std::string & yaml_base_dir);

  static int countPseudojointsFromAssembly(
    const std::string & yaml_base_dir);
};

}  // namespace smm_controllers