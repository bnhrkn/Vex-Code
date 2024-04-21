#pragma once

#include <string>
namespace auton {
enum class AutonMode : int {
  disabled = 0,
  close_quals = 1,
  far_quals = 2,
  close_finals = 3,
  far_finals = 4,
  prog_skills = 5
};
const std::string autonNames =
    "Disabled\nQuals Near\nQuals Far\nFinals Near\nFinals Far\nProg Skills\n";

}  // namespace auton
