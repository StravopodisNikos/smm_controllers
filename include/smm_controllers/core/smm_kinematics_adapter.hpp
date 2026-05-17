#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "smm_screws/core/RobotAbstractBaseNdof.h"
#include "smm_screws/core/RobotContextNdof.h"
#include "smm_screws/core/ScrewsKinematicsNdof.h"

namespace smm_controllers
{

class SmmKinematicsAdapter
{
public:
  SmmKinematicsAdapter() = default;

  bool initialize(const std::string & yaml_base_dir);

  int dof() const;

  bool computeTcpKinematics(
    const Eigen::VectorXd & q,
    const Eigen::VectorXd & qdot,
    Eigen::Vector3d & tcp_position,
    Eigen::Vector3d & tcp_velocity,
    Eigen::MatrixXd & translational_jacobian);

private:
  std::unique_ptr<RobotContextNdof> robot_context_ndof_;

  int dof_{0};

  std::vector<float> q_float_;
  std::vector<float> qdot_float_;
  std::vector<float> qddot_zero_;

  static std::unique_ptr<RobotAbstractBaseNdof> createRobotFromYaml(
    const std::string & yaml_base_dir);

  static int countPseudojointsFromAssembly(
    const std::string & yaml_base_dir);
};

}  // namespace smm_controllers