#include "project/wings.hpp"

Wings::Wings(pros::adi::DigitalOut left, pros::adi::DigitalOut right)
    : left(left), right(right) {
  left.set_value(0);
  right.set_value(0);
}

void Wings::toggleExtended(Wing wing, std::optional<bool> state) {
  switch (wing) {
    case Wing::left:
      leftState = state.value_or(!leftState);
      left.set_value(static_cast<std::int32_t>(leftState));

      break;
    case Wing::right:
      rightState = state.value_or(!rightState);
      right.set_value(static_cast<std::int32_t>(rightState));
      break;
  }
}

bool Wings::getState(Wing wing) const {
  switch (wing) {
    case Wing::left:
      return leftState;
    case Wing::right:
      return rightState;
  }
}