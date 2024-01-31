#pragma once
#include "main.h"

class Wings {
 public:
  enum class Wing { left, right };
  Wings(pros::adi::DigitalOut left, pros::adi::DigitalOut right);
  void toggleExtended(Wing wing, std::optional<bool> = std::nullopt);
  [[nodiscard]] bool getState(Wing wing) const;

 private:
  pros::adi::DigitalOut left, right;
  bool leftState = false, rightState = false;
};