#include "smm_controllers/core/smm_controller_utils.hpp"

namespace smm_controllers
{

std::vector<std::string> make_interface_names(
  const std::vector<std::string> & joint_names,
  const std::string & interface_name)
{
  std::vector<std::string> names;
  names.reserve(joint_names.size());

  for (const auto & joint_name : joint_names) {
    names.push_back(joint_name + "/" + interface_name);
  }

  return names;
}

bool write_vector_to_command_interfaces(
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
  const std::vector<std::string> & joint_names,
  const Eigen::VectorXd & command_vector,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock,
  const std::string & command_name)
{
  const auto n = joint_names.size();

  if (command_interfaces.size() != n) {
    RCLCPP_ERROR(
      logger,
      "Cannot write %s command. Expected %zu command interfaces, got %zu.",
      command_name.c_str(),
      n,
      command_interfaces.size());
    return false;
  }

  if (command_vector.size() != static_cast<Eigen::Index>(n)) {
    RCLCPP_ERROR(
      logger,
      "Cannot write %s command. Expected vector size %zu, got %ld.",
      command_name.c_str(),
      n,
      command_vector.size());
    return false;
  }

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    const bool command_written =
      command_interfaces[i].set_value(command_vector(idx));

    if (!command_written) {
      RCLCPP_ERROR_THROTTLE(
        logger,
        clock,
        1000,
        "Failed to write %s command for joint '%s'.",
        command_name.c_str(),
        joint_names[i].c_str());

      return false;
    }
  }

  return true;
}

void fill_joint_error_state_msg(
  sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & joint_names,
  const Eigen::VectorXd & position_error,
  const Eigen::VectorXd & velocity_error,
  const Eigen::VectorXd & effort_command)
{
  const auto n = joint_names.size();

  msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  msg.name = joint_names;

  msg.position.resize(n, 0.0);
  msg.velocity.resize(n, 0.0);
  msg.effort.resize(n, 0.0);

  for (size_t i = 0; i < n; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    msg.position[i] =
      idx < position_error.size() ? position_error(idx) : 0.0;

    msg.velocity[i] =
      idx < velocity_error.size() ? velocity_error(idx) : 0.0;

    msg.effort[i] =
      idx < effort_command.size() ? effort_command(idx) : 0.0;
  }
}

void publish_scalar_error_topics(
  const std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> & pubs,
  std::vector<std_msgs::msg::Float64> & msgs,
  const Eigen::VectorXd & values)
{
  const auto n = pubs.size();

  if (msgs.size() != n) {
    msgs.resize(n);
  }

  for (size_t i = 0; i < n; ++i) {
    if (!pubs[i]) {
      continue;
    }

    const auto idx = static_cast<Eigen::Index>(i);

    msgs[i].data = idx < values.size() ? values(idx) : 0.0;
    pubs[i]->publish(msgs[i]);
  }
}

}  // namespace smm_controllers