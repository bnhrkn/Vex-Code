#include "project/catapult.hpp"
#include <limits>
#include <utility>
#include "mp-units/core.h"
using namespace okapi::literals;

Catapult::Catapult(pros::Motor motor,
                   pros::Rotation rotation,
                   pros::Distance distance)
    : motor(std::move(motor)),
      rotation(std::move(rotation)),
      distance(std::move(distance)),
      internalTask([this]() { taskFunction(); }) {
  if (abs(getAngle() - 45_deg) < 5_deg) {
    ready = true;
  }
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
};

Catapult::~Catapult() {
  internalTask.remove();
  motor.move(0);
}
void Catapult::setArmMode(bool armMode, uint32_t shotNum) {
  alwaysArm = armMode;
}
bool Catapult::getArmMode() {
  return alwaysArm;
}

void Catapult::taskFunction() {
  int closeTime = 0;
  int readyTime = 0;
  // bool countedShot = false;
  pros::c::adi_pin_mode(4, OUTPUT);
  auto led = pros::adi::DigitalOut(4);
  while (pros::Task::notify_take(true, 0) == 0U) {
    constexpr auto readyAngle = 30_deg;
    constexpr auto doneAngle = 5_deg;
    constexpr auto distMinThres = 30;
    constexpr auto distReadyThres = 250;
    auto distReading = distance.get();

    if (getAngle() > readyAngle) {
      armed = true;
    } else if (getAngle() < doneAngle) {
      armed = false;
    }

    if (distReading <= distMinThres) {
      closeTime += 10;
    } else {
      closeTime = 0;
    }

    // if (armed && distReading < distReadyThres) {
    //   readyTime += 10;
    // } else {
    //   readyTime = 0;
    // }
    bool movingUp = rotation.get_velocity() < -100;
    bool shouldArm =
        !movingUp && !armed && (distReading <= distReadyThres || alwaysArm);
    bool shouldFire = !movingUp && closeTime > 50;

    if (shouldFire || shouldArm) {
      motor.move_voltage(12000);
      led.set_value(0);
      // std::cout << "Drawing back. Velocity: " <<
      // rotation.get_velocity()
      //           << " Voltage: " << motor.get_voltage() << "\n";
    } else if (movingUp) {
      motor.move_velocity(0);
      led.set_value(1);
      pros::delay(50);
    } else {
      motor.move_voltage(0);
      led.set_value(0);

      // std::cout << "Not Drawing Back.\n";
    }

    // if (rotation.get_velocity() < -250) {
    //   motor.move_velocity(0);
    //   std::cout << "Position" << rotation.get_position() << "\n";
    // }
    pros::delay(10);
  }
}

// void Catapult::fire() {
//   shouldArm = false;
//   shouldFire = !shouldFire;
// }

// void Catapult::arm() {
//   shouldArm = true;
// }

bool Catapult::isArmed() {
  return armed;
}

// bool Catapult::isReady() {
//   return ready;
// }

int Catapult::getShotNum() {
  return shotNum;
}

okapi::QAngle Catapult::getAngle() {
  return okapi::OdomMath::constrainAngle180(rotation.get_position() / 100.0 *
                                            1_deg);
}