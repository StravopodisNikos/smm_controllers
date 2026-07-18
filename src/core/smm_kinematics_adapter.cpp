#include "smm_controllers/core/smm_kinematics_adapter.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "smm_screws/robot_parameters.h"

namespace smm_controllers
{

bool SmmKinematicsAdapter::initialize(const std::string & yaml_base_dir)
{
  try {
    auto robot = createRobotFromYaml(yaml_base_dir);

    if (!robot) {
      std::cerr << "[SmmKinematicsAdapter::initialize] Failed to create robot from YAML.\n";
      return false;
    }

    robot_context_ndof_ =
      std::make_unique<RobotContextNdof>(std::move(robot));

    if (!robot_context_ndof_->initializeSharedLib()) {
      std::cerr << "[SmmKinematicsAdapter::initialize] "
                << "RobotContextNdof::initializeSharedLib() failed.\n";
      return false;
    }

    RobotAbstractBaseNdof * robot_ptr = robot_context_ndof_->get_robot();

    if (!robot_ptr) {
      std::cerr << "[SmmKinematicsAdapter::initialize] "
                << "RobotContextNdof::get_robot() returned nullptr.\n";
      return false;
    }

    dof_ = robot_ptr->get_DOF();

    if (dof_ <= 0 || dof_ > robot_params::MAX_DOF) {
      std::cerr << "[SmmKinematicsAdapter::initialize] Invalid DOF = "
                << dof_ << "\n";
      return false;
    }

    q_float_.assign(static_cast<size_t>(dof_), 0.0f);
    qdot_float_.assign(static_cast<size_t>(dof_), 0.0f);
    qddot_zero_.assign(static_cast<size_t>(dof_), 0.0f);

    std::cout << "[SmmKinematicsAdapter::initialize] Initialized. DOF = "
              << dof_ << "\n";

    return true;
  }
  catch (const std::exception & e) {
    std::cerr << "[SmmKinematicsAdapter::initialize] Exception: "
              << e.what() << "\n";
    return false;
  }
}

int SmmKinematicsAdapter::dof() const
{
  return dof_;
}

bool SmmKinematicsAdapter::computeTcpKinematics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qdot,
  Eigen::Vector3d & tcp_position,
  Eigen::Vector3d & tcp_velocity,
  Eigen::MatrixXd & translational_jacobian)
{
  if (!robot_context_ndof_) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpKinematics] "
              << "Adapter is not initialized.\n";
    return false;
  }

  if (dof_ <= 0) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpKinematics] Invalid DOF.\n";
    return false;
  }

  if (q.size() != dof_ || qdot.size() != dof_) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpKinematics] Size mismatch. "
              << "Expected q/qdot size " << dof_
              << ", got q=" << q.size()
              << ", qdot=" << qdot.size() << "\n";
    return false;
  }

  for (int i = 0; i < dof_; ++i) {
    q_float_[static_cast<size_t>(i)] =
      static_cast<float>(q(static_cast<Eigen::Index>(i)));

    qdot_float_[static_cast<size_t>(i)] =
      static_cast<float>(qdot(static_cast<Eigen::Index>(i)));

    qddot_zero_[static_cast<size_t>(i)] = 0.0f;
  }

  try {
    auto & kin = robot_context_ndof_->get_kinematics();

    // -----------------------------------------------------------------------
    // 1) Update internal kinematic state.
    //    This is needed because velocity/Jacobian functions use internal
    //    joint velocity storage.
    // -----------------------------------------------------------------------
    kin.updateJointState(
      q_float_.data(),
      qdot_float_.data(),
      qddot_zero_.data());

    // -----------------------------------------------------------------------
    // 2) Forward kinematics to current TCP pose.
    // -----------------------------------------------------------------------
    kin.ForwardKinematicsTCP(q_float_.data());

    const Eigen::Isometry3f & g_tcp = kin.getTcpPose();

    tcp_position =
      g_tcp.translation().cast<double>();

    // -----------------------------------------------------------------------
    // 3) Operational/hybrid TCP Jacobian.
    //
    //    Your convention:
    //      Jop = hybrid TCP Jacobian
    //      twist order = [v; w]
    //
    //    Therefore:
    //      Jv = Jop.topRows(3)
    // -----------------------------------------------------------------------
    kin.computeBodyJacobiansFrames1();
    kin.computeHybridJacobianTCP();

    if (!kin.hasOperationalJacobianTCP()) {
      std::cerr << "[SmmKinematicsAdapter::computeTcpKinematics] "
                << "Operational Jacobian is not valid.\n";
      return false;
    }

    const Eigen::Matrix<float, 6, Eigen::Dynamic> Jop =
      kin.getOperationalJacobianTCP();

    if (Jop.rows() != 6 || Jop.cols() != dof_) {
      std::cerr << "[SmmKinematicsAdapter::computeTcpKinematics] "
                << "Invalid Jop dimensions: "
                << Jop.rows() << "x" << Jop.cols()
                << ", expected 6x" << dof_ << "\n";
      return false;
    }

    translational_jacobian =
      Jop.topRows(3).cast<double>();

    // -----------------------------------------------------------------------
    // 4) TCP translational velocity.
    //
    //    Since Jop is [v; w] and expressed in the base/inertial frame:
    //      xdot = Jv * qdot
    // -----------------------------------------------------------------------
    tcp_velocity =
      translational_jacobian * qdot;

    return true;
  }
  catch (const std::exception & e) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpKinematics] Exception: "
              << e.what() << "\n";
    return false;
  }
}

bool SmmKinematicsAdapter::computeTcpPoseKinematics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qdot,
  Eigen::Vector3d & tcp_position,
  Eigen::Matrix3d & tcp_orientation,
  Eigen::Vector3d & tcp_linear_velocity,
  Eigen::Vector3d & tcp_angular_velocity,
  Eigen::MatrixXd & operational_jacobian)
{
  if (!robot_context_ndof_) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
              << "Adapter is not initialized.\n";
    return false;
  }

  if (dof_ <= 0) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
              << "Invalid DOF.\n";
    return false;
  }

  if (q.size() != dof_ || qdot.size() != dof_) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
              << "Size mismatch. Expected q/qdot size " << dof_
              << ", got q=" << q.size()
              << ", qdot=" << qdot.size() << "\n";
    return false;
  }

  for (int i = 0; i < dof_; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    q_float_[static_cast<size_t>(i)] =
      static_cast<float>(q(idx));

    qdot_float_[static_cast<size_t>(i)] =
      static_cast<float>(qdot(idx));

    qddot_zero_[static_cast<size_t>(i)] = 0.0f;
  }

  try {
    auto & kin = robot_context_ndof_->get_kinematics();

    // -----------------------------------------------------------------------
    // 1) Update internal state.
    //
    // q, qdot, qddot are stored internally by smm_screws and are needed by
    // the velocity/Jacobian-related computations.
    // -----------------------------------------------------------------------
    kin.updateJointState(
      q_float_.data(),
      qdot_float_.data(),
      qddot_zero_.data());

    // -----------------------------------------------------------------------
    // 2) Forward kinematics to TCP.
    // -----------------------------------------------------------------------
    kin.ForwardKinematicsTCP(q_float_.data());

    const Eigen::Isometry3f & g_tcp = kin.getTcpPose();

    tcp_position =
      g_tcp.translation().cast<double>();

    tcp_orientation =
      g_tcp.linear().cast<double>();

    // -----------------------------------------------------------------------
    // 3) Full operational/hybrid TCP Jacobian.
    //
    // Convention from smm_screws:
    //   Jop is 6 x n
    //   twist order is [v; w]
    //   Jop is measured at TCP and expressed in base/inertial frame.
    // -----------------------------------------------------------------------
    kin.computeBodyJacobiansFrames1();
    kin.computeHybridJacobianTCP();

    if (!kin.hasOperationalJacobianTCP()) {
      std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
                << "Operational Jacobian is not valid.\n";
      return false;
    }

    const Eigen::Matrix<float, 6, Eigen::Dynamic> Jop =
      kin.getOperationalJacobianTCP();

    if (Jop.rows() != 6 || Jop.cols() != dof_) {
      std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
                << "Invalid Jop dimensions: "
                << Jop.rows() << "x" << Jop.cols()
                << ", expected 6x" << dof_ << "\n";
      return false;
    }

    operational_jacobian =
      Jop.cast<double>();

    // -----------------------------------------------------------------------
    // 4) TCP hybrid twist.
    //
    // Since twist order is [v; w]:
    //   first 3 elements  -> TCP linear velocity
    //   last 3 elements   -> TCP angular velocity
    // -----------------------------------------------------------------------
    const Eigen::VectorXd tcp_twist =
      operational_jacobian * qdot;

    if (tcp_twist.size() != 6) {
      std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
                << "Invalid TCP twist size: " << tcp_twist.size()
                << ", expected 6.\n";
      return false;
    }

    tcp_linear_velocity =
      tcp_twist.segment<3>(0);

    tcp_angular_velocity =
      tcp_twist.segment<3>(3);

    return true;
  }
  catch (const std::exception & e) {
    std::cerr << "[SmmKinematicsAdapter::computeTcpPoseKinematics] "
              << "Exception: " << e.what() << "\n";
    return false;
  }
}

std::unique_ptr<RobotAbstractBaseNdof>
SmmKinematicsAdapter::createRobotFromYaml(const std::string & yaml_base_dir)
{
  const int pseudo_count = countPseudojointsFromAssembly(yaml_base_dir);

  if (pseudo_count == 0) {
    return std::make_unique<FixedStructureNdof>(yaml_base_dir);
  }

  switch (pseudo_count) {
    case 2:
      return std::make_unique<Structure2PseudosNdof>(yaml_base_dir);

    case 3:
      return std::make_unique<Structure3PseudosNdof>(yaml_base_dir);

    case 4:
      return std::make_unique<Structure4PseudosNdof>(yaml_base_dir);

    case 5:
      return std::make_unique<Structure5PseudosNdof>(yaml_base_dir);

    case 6:
      return std::make_unique<Structure6PseudosNdof>(yaml_base_dir);

    default:
      throw std::runtime_error(
        "Unsupported pseudojoint count: " + std::to_string(pseudo_count));
  }
}

int SmmKinematicsAdapter::countPseudojointsFromAssembly(
  const std::string & yaml_base_dir)
{
  std::string base_dir = yaml_base_dir;

  if (!base_dir.empty() &&
      base_dir.back() != '/' &&
      base_dir.back() != '\\')
  {
    base_dir += "/";
  }

  const std::string assembly_path = base_dir + "assembly.yaml";

  YAML::Node assembly = YAML::LoadFile(assembly_path);

  auto get_or_default = [&](const char * key, int default_value) -> int {
    if (assembly[key]) {
      return assembly[key].as<int>();
    }
    return default_value;
  };

  const int s2 = get_or_default("s2", 9);
  const int s3 = get_or_default("s3", 9);
  const int s5 = get_or_default("s5", 9);
  const int s6 = get_or_default("s6", 9);
  const int s8 = get_or_default("s8", 9);
  const int s9 = get_or_default("s9", 9);

  int pseudo_count = 0;

  if (s2 != 9) {
    ++pseudo_count;
  }
  if (s3 != 9) {
    ++pseudo_count;
  }
  if (s5 != 9) {
    ++pseudo_count;
  }
  if (s6 != 9) {
    ++pseudo_count;
  }
  if (s8 != 9) {
    ++pseudo_count;
  }
  if (s9 != 9) {
    ++pseudo_count;
  }

  return pseudo_count;
}

}  // namespace smm_controllers