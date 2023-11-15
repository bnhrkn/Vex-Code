#include "project/catapult.hpp"
#include <limits>

Catapult::Catapult(std::vector<std::int8_t> motorPorts,
                   pros::Rotation rotation,
                   pidf gains)
    : controller(gains),
      motors(std::move(motorPorts)),
      rotation(std::move(rotation)),
      motorTask([this]() { motorLoop(); }){};

Catapult::~Catapult() {
  motorTask.remove();
  motors.move(0);
}

Catapult::CATAPULT_STATE Catapult::getState() {
  constexpr double lowerStopDeg = 45;
  constexpr double jamPointDeg = 55;
  constexpr double minShootSpeed = 500;
  constexpr double minLoadSpeed = 500;
  constexpr double positionTolerance = 500;
  // Moving up fast
  if (rotation.get_velocity() + minShootSpeed < 0) {
    return CATAPULT_STATE::shooting;
  }
  // Moving down fast
  // TODO this can occur at the start of a shot cycle
  if (rotation.get_velocity() - minLoadSpeed > 0) {
    return CATAPULT_STATE::loading;
  }
  // Appx. near low stop
  if (rotation.get_position() + positionTolerance > lowerStopDeg * 100) {
    return CATAPULT_STATE::armed;
  }
  // Appx. near high stop
  if (rotation.get_position() - positionTolerance < 0) {
    return CATAPULT_STATE::idle;
  }
  // Really too low and stuck
  if (rotation.get_position() - 100 < jamPointDeg * 100 &&
      motors.get_efficiency(0) < 2) {
    return CATAPULT_STATE::jam;
  }
  return CATAPULT_STATE::unknown;
}

void Catapult::motorLoop() {
  constexpr double loadPositionDeg = 45;
  motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  motors.set_gearing(pros::E_MOTOR_GEAR_RED);
  motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  // auto volts = [](okapi::QAngle angle) -> double {
  //   return angle.convert(okapi::degree) * 1.0;
  // };
  while (true) {
    while (rotation.get_position() < loadPositionDeg * 100) {
      motors.move(127);
      pros::delay(10);
    }
    motors.move(0);
    std::cout << "Armed\n";
    state = Catapult::CATAPULT_STATE::armed;

    pros::Task::notify_take(true, std::numeric_limits<std::uint32_t>::max());
    std::cout << "Shooting\n";
    state = Catapult::CATAPULT_STATE::shooting;

    motors.move(127);
    // while (rotation.get_velocity() < 0) {
    //   pros::delay(1);
    // }
    // motors.move_velocity(0);
    // while (std::abs(rotation.get_position()) < 5) {
    //   pros::delay(1);
    // }
    state = Catapult::CATAPULT_STATE::loading;
  }
}

bool Catapult::fire() {
  if (state != CATAPULT_STATE::armed) {
    return false;
  }
  motorTask.notify();
  return true;
}

bool Catapult::isArmed() {
  return state == Catapult::CATAPULT_STATE::armed;
}