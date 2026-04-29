#pragma once

#include <string>
#include <vector>

namespace smm_controllers
{

std::vector<std::string> make_interface_names(
  const std::vector<std::string> & joint_names,
  const std::string & interface_name);

}  // namespace smm_controllers