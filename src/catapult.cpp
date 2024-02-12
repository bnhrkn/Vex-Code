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
};

Catapult::~Catapult() {
  internalTask.remove();
  motor.move(0);
}

void Catapult::taskFunction() {
  int closeTime = 0;
  bool countedShot = false;
  while (pros::Task::notify_take(true, 0) == 0U) {
    constexpr auto distMinThres = 30;
    auto distReading = distance.get();

    if (getAngle() > 50_deg) {
      armed = true;
    } else if (getAngle() < 15_deg) {
      armed = false;
    }

    if (distReading <= distMinThres) {
      closeTime += 10;
    } else {
      closeTime = 0;
    }

    if (rotation.get_velocity() < 1000 && !countedShot) {
      shotNum++;
      countedShot = true;
      std::cout << shotNum << "\n";
    } else if (rotation.get_velocity() > 0) {
      countedShot = false;
    }

    if ((getAngle() < 55_deg && !armed) || closeTime > 50) {
      motor.move_voltage(12000);
      // std::cout << rotation.get_velocity() << "\n";
    } else {
      motor.move_voltage(0);
    }
    pros::delay(10);

    //   // std::cout << "Shoot Drawing\n";
    //   armed = false;
    //   ready = false;

    //   int numZeroEfficiency = 0;
    //   while (sensor.get_velocity() > -100) {
    //     if (motor.get_efficiency() == 0) {
    //       numZeroEfficiency++;
    //     } else {
    //       numZeroEfficiency = 0;
    //     }
    //     if (numZeroEfficiency > 10) {
    //       std::cout << "Catapult Liamed Out\n";
    //       motor.move_voltage(0);
    //       ready = true;
    //     } else {
    //       motor.move_voltage(10000);
    //     }
    //     pros::delay(10);
    //   }

    //   // motor.move_voltage(0);
    //   motor.move_velocity(0);
    //   motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    //   while (getAngle() > 5_deg) {
    //     // Wait for catapult to go up
    //     pros::delay(10);
    //   }
    //   shouldFire = false;
    // } else if (shouldArm) {
    //   // std::cout << "Arm Drawing\n";
    //   while (getAngle() < 56_deg) {
    //     motor.move_voltage(10000);
    //     pros::delay(1);
    //   }
    //   motor.move_voltage(0);
    //   armed = true;
    //   ready = true;
    //   shouldArm = false;
    // } else if (abs(getAngle() - 45_deg) > 5_deg) {
    //   // std::cout << "Idle Drawing\n";
    //   while (getAngle() < 45_deg) {
    //     motor.move_voltage(10000);
    //     pros::delay(10);
    //   }
    //   ready = true;
    //   motor.move_voltage(0);
  }
}

void Catapult::fire() {
  shouldArm = false;
  shouldFire = !shouldFire;
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

int Catapult::getShotNUm() {
  return shotNum;
}

okapi::QAngle Catapult::getAngle() {
  return okapi::OdomMath::constrainAngle180(rotation.get_position() / 100.0 *
                                            1_deg);
}