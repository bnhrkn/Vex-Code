#pragma once

#include <string>
namespace auton {
enum class AutonMode : int {
  disabled,
  close_quals,
  far_quals,
  close_finals,
  far_finals,
  prog_skills
};
const std::string autonNames =
    "Disabled\nQuals Near\nQuals Far\nFinals Near\nFinals Far\nProg Skills\n";

}  // namespace auton
