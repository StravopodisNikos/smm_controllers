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

}  // namespace smm_controllers