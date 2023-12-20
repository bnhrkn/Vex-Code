#include "project/catapult.hpp"
#include <limits>
#include <utility>
using namespace okapi::literals;

Catapult::Catapult(pros::Motor motor, pros::Rotation rotation)
    : motor(std::move(motor)),
      sensor(std::move(rotation)),
      internalTask([this]() { taskFunction(); }) {
  if (abs(getAngle() - 45_deg) < 5_deg) {
    ready = true;
  }
};

Catapult::~Catapult() {
  internalTask.remove();
  motor.move(0);
}

void Catapult::taskFunction() {
  while (pros::Task::notify_take(true, 10) == 0U) {
    if (shouldFire) {
      // std::cout << "Shoot Drawing\n";
      armed = false;
      ready = false;

      int numZeroEfficiency = 0;
      while (sensor.get_velocity() > -100) {
        if (motor.get_efficiency() == 0) {
          numZeroEfficiency++;
        } else {
          numZeroEfficiency = 0;
        }
        if (numZeroEfficiency > 10) {
          std::cout << "Catapult Liamed Out\n";
          motor.move_voltage(0);
          ready = true;
        } else {
          motor.move_voltage(10000);
        }
        pros::delay(10);
      }

      // motor.move_voltage(0);
      motor.move_velocity(0);
      motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      while (getAngle() > 5_deg) {
        // Wait for catapult to go up
        pros::delay(10);
      }
      shouldFire = false;
    } else if (shouldArm) {
      // std::cout << "Arm Drawing\n";
      while (getAngle() < 56_deg) {
        motor.move_voltage(10000);
        pros::delay(1);
      }
      motor.move_voltage(0);
      armed = true;
      ready = true;
      shouldArm = false;
    } else if (abs(getAngle() - 45_deg) > 5_deg) {
      // std::cout << "Idle Drawing\n";
      while (getAngle() < 45_deg) {
        motor.move_voltage(10000);
        pros::delay(10);
      }
      ready = true;
      motor.move_voltage(0);
    }
  }
}

void Catapult::fire() {
  shouldArm = false;
  shouldFire = true;
}

void Catapult::arm() {
  shouldArm = true;
}

bool Catapult::isArmed() {
  return armed;
}

bool Catapult::isReady() {
  return ready;
}

okapi::QAngle Catapult::getAngle() {
  return okapi::OdomMath::constrainAngle180(sensor.get_position() / 100.0 *
                                            1_deg);
}