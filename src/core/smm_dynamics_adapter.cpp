#include "smm_controllers/core/smm_dynamics_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace smm_controllers
{

ScrewsDynamicsNdof::DynamicsRepresentation
SmmDynamicsAdapter::parseRepresentation(const std::string & representation)
{
  std::string rep = representation;

  std::transform(
    rep.begin(),
    rep.end(),
    rep.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (rep == "body") {
    return ScrewsDynamicsNdof::DynamicsRepresentation::BODY;
  }

  if (rep == "spatial") {
    return ScrewsDynamicsNdof::DynamicsRepresentation::SPATIAL;
  }

  throw std::runtime_error(
    "[SmmDynamicsAdapter] Invalid gravity_representation. Use 'body' or 'spatial'.");
}

int SmmDynamicsAdapter::countPseudojointsFromAssembly(
  const std::string & yaml_base_dir)
{
  std::string base_dir = yaml_base_dir;

  if (!base_dir.empty() && base_dir.back() != '/' && base_dir.back() != '\\') {
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

std::unique_ptr<RobotAbstractBaseNdof>
SmmDynamicsAdapter::createRobotFromYaml(const std::string & yaml_base_dir)
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
        "[SmmDynamicsAdapter] Unsupported pseudojoint count: " +
        std::to_string(pseudo_count));
  }
}

bool SmmDynamicsAdapter::initialize(
  const std::string & yaml_base_dir,
  const std::string & gravity_representation,
  const std::string & operational_dynamics_method,
  const double operational_damping)
{
  try {
    gravity_representation_ = parseRepresentation(gravity_representation);

    operational_dynamics_method_ =
      parseOperationalDynamicsMethod(operational_dynamics_method);

    operational_damping_ =
      static_cast<float>(std::max(operational_damping, 1.0e-6));

    auto robot = createRobotFromYaml(yaml_base_dir);

    robot_context_ndof_ = std::make_unique<RobotContextNdof>(std::move(robot));

    if (!robot_context_ndof_->initializeSharedLib()) {
      std::cerr << "[SmmDynamicsAdapter] RobotContextNdof::initializeSharedLib() failed.\n";
      return false;
    }

    RobotAbstractBaseNdof * robot_ptr = robot_context_ndof_->get_robot();

    if (!robot_ptr) {
      std::cerr << "[SmmDynamicsAdapter] RobotContextNdof::get_robot() returned nullptr.\n";
      return false;
    }

    dof_ = robot_ptr->get_DOF();

    if (dof_ <= 0 || dof_ > robot_params::MAX_DOF) {
      std::cerr << "[SmmDynamicsAdapter] Invalid DOF = " << dof_ << "\n";
      return false;
    }

    auto & dyn = robot_context_ndof_->get_dynamics();
    dyn.initializeLinkMassMatrices();

    q_float_.assign(static_cast<std::size_t>(dof_), 0.0f);
    qdot_float_.assign(static_cast<std::size_t>(dof_), 0.0f);
    qddot_zero_.assign(static_cast<std::size_t>(dof_), 0.0f);

    return true;
  } catch (const std::exception & e) {
    std::cerr << "[SmmDynamicsAdapter] initialize() failed: " << e.what() << "\n";
    return false;
  }
}

int SmmDynamicsAdapter::dof() const
{
  return dof_;
}

bool SmmDynamicsAdapter::computeGravity(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qdot,
  Eigen::VectorXd & gravity)
{
  if (!robot_context_ndof_) {
    return false;
  }

  if (q.size() != dof_ || qdot.size() != dof_) {
    return false;
  }

  auto & dyn = robot_context_ndof_->get_dynamics();

  for (int i = 0; i < dof_; ++i) {
    q_float_[static_cast<std::size_t>(i)] = static_cast<float>(q(i));
    qdot_float_[static_cast<std::size_t>(i)] = static_cast<float>(qdot(i));
    qddot_zero_[static_cast<std::size_t>(i)] = 0.0f;
  }

  dyn.updateJointState(
    q_float_.data(),
    qdot_float_.data(),
    qddot_zero_.data());

  dyn.ForwardKinematicsCOM(q_float_.data());

  if (gravity_representation_ == ScrewsDynamicsNdof::DynamicsRepresentation::BODY) {
    dyn.computeBodyCOMJacobiansFrames();
  }

  const Eigen::Matrix<float, Eigen::Dynamic, 1> gravity_float =
    dyn.GravityVector(gravity_representation_);

  if (gravity_float.size() != dof_) {
    return false;
  }

  if (gravity.size() != dof_) {
    gravity.resize(dof_);
  }

  for (int i = 0; i < dof_; ++i) {
    gravity(i) = static_cast<double>(gravity_float(i));
  }

  return true;
}

bool SmmDynamicsAdapter::computeJointDynamics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qdot,
  Eigen::MatrixXd & mass_matrix,
  Eigen::MatrixXd & coriolis_matrix,
  Eigen::VectorXd & gravity)
{
  if (!robot_context_ndof_) {
    return false;
  }

  if (q.size() != dof_ || qdot.size() != dof_) {
    return false;
  }

  auto & dyn = robot_context_ndof_->get_dynamics();

  for (int i = 0; i < dof_; ++i) {
    q_float_[static_cast<std::size_t>(i)] = static_cast<float>(q(i));
    qdot_float_[static_cast<std::size_t>(i)] = static_cast<float>(qdot(i));
    qddot_zero_[static_cast<std::size_t>(i)] = 0.0f;
  }

  dyn.updateJointState(
    q_float_.data(),
    qdot_float_.data(),
    qddot_zero_.data());

  const auto rep = gravity_representation_;

  if (rep == ScrewsDynamicsNdof::DynamicsRepresentation::BODY) {
    dyn.ForwardKinematicsTCP(q_float_.data());
    dyn.computeBodyJacobiansFrames2();

    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M_f =
      dyn.MassMatrix(
        rep,
        ScrewsDynamicsNdof::BodyFrameSelection::JOINT);

    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C_f =
      dyn.CoriolisMatrix(rep);

    dyn.ForwardKinematicsCOM(q_float_.data());
    dyn.computeBodyCOMJacobiansFrames();

    const Eigen::Matrix<float, Eigen::Dynamic, 1> G_f =
      dyn.GravityVector(rep);

    mass_matrix = M_f.cast<double>();
    coriolis_matrix = C_f.cast<double>();
    gravity = G_f.cast<double>();
  } else {
    dyn.ForwardKinematicsTCP(q_float_.data());

    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> M_f =
      dyn.MassMatrix(rep);

    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> C_f =
      dyn.CoriolisMatrix(rep);

    dyn.ForwardKinematicsCOM(q_float_.data());

    const Eigen::Matrix<float, Eigen::Dynamic, 1> G_f =
      dyn.GravityVector(rep);

    mass_matrix = M_f.cast<double>();
    coriolis_matrix = C_f.cast<double>();
    gravity = G_f.cast<double>();
  }

  return (
    mass_matrix.rows() == dof_ &&
    mass_matrix.cols() == dof_ &&
    coriolis_matrix.rows() == dof_ &&
    coriolis_matrix.cols() == dof_ &&
    gravity.size() == dof_);
}

bool SmmDynamicsAdapter::computeNonredundantOperationalDynamics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qdot,
  Eigen::MatrixXd & operational_mass_matrix,
  Eigen::VectorXd & operational_coriolis_vector,
  Eigen::VectorXd & operational_gravity_vector,
  Eigen::MatrixXd & square_operational_jacobian,
  Eigen::MatrixXd & square_operational_jacobian_dot)
{
  if (!robot_context_ndof_) {
    std::cerr << "[SmmDynamicsAdapter] computeNonredundantOperationalDynamics() failed: "
              << "adapter is not initialized.\n";
    return false;
  }

  if (q.size() != dof_ || qdot.size() != dof_) {
    std::cerr << "[SmmDynamicsAdapter] computeNonredundantOperationalDynamics() failed: "
              << "invalid q/qdot size.\n";
    return false;
  }

  try {
    auto & dyn = robot_context_ndof_->get_dynamics();

    for (int i = 0; i < dof_; ++i) {
      q_float_[static_cast<std::size_t>(i)] =
        static_cast<float>(q(static_cast<Eigen::Index>(i)));

      qdot_float_[static_cast<std::size_t>(i)] =
        static_cast<float>(qdot(static_cast<Eigen::Index>(i)));

      qddot_zero_[static_cast<std::size_t>(i)] = 0.0f;
    }

    dyn.updateJointState(
      q_float_.data(),
      qdot_float_.data(),
      qddot_zero_.data());

    /*
     * The adapter chooses the Jacobian computation pipeline.
     * Dynamics math remains inside ScrewsDynamicsNdof.
     */
    dyn.ForwardKinematicsTCP(q_float_.data());
    dyn.computeBodyJacobiansFrames2();
    dyn.computeHybridJacobianTCP();
    dyn.computeHybridVelocityTwistTCP();
    dyn.computeDtHybridJacobianTCP();

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Mx_f;
    Eigen::Matrix<float, Eigen::Dynamic, 1> Cx_f;
    Eigen::Matrix<float, Eigen::Dynamic, 1> Gx_f;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jx_f;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> dJx_f;

    if (!dyn.computeOperationalSpaceDynamics(
        gravity_representation_,
        operational_dynamics_method_,
        operational_damping_,
        Mx_f,
        Cx_f,
        Gx_f,
        Jx_f,
        dJx_f))
    {
      std::cerr << "[SmmDynamicsAdapter] computeNonredundantOperationalDynamics() failed: "
                << "ScrewsDynamicsNdof::computeOperationalSpaceDynamics() failed.\n";
      return false;
    }

    operational_mass_matrix = Mx_f.cast<double>();
    operational_coriolis_vector = Cx_f.cast<double>();
    operational_gravity_vector = Gx_f.cast<double>();
    square_operational_jacobian = Jx_f.cast<double>();
    square_operational_jacobian_dot = dJx_f.cast<double>();

    return true;
  } catch (const std::exception & e) {
    std::cerr << "[SmmDynamicsAdapter] computeNonredundantOperationalDynamics() failed: "
              << e.what() << "\n";
    return false;
  }
}

ScrewsDynamicsNdof::OperationalDynamicsMethod
SmmDynamicsAdapter::parseOperationalDynamicsMethod(const std::string & method)
{
  std::string m = method;

  std::transform(
    m.begin(),
    m.end(),
    m.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (m == "exact") {
    return ScrewsDynamicsNdof::OperationalDynamicsMethod::EXACT;
  }

  if (m == "damped") {
    return ScrewsDynamicsNdof::OperationalDynamicsMethod::DAMPED;
  }

  if (m == "exact_with_damped_fallback" || m == "fallback") {
    return ScrewsDynamicsNdof::OperationalDynamicsMethod::EXACT_WITH_DAMPED_FALLBACK;
  }

  throw std::runtime_error(
    "[SmmDynamicsAdapter] Invalid operational_dynamics_method. "
    "Use 'exact', 'damped', or 'exact_with_damped_fallback'.");
}

}  // namespace smm_controllers

