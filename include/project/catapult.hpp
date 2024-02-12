#pragma once
#include <utility>

#include "main.h"
#include "project/PIDF.hpp"
#include "project/algorithms.hpp"

class Catapult {
 public:
  Catapult(pros::Motor motor, pros::Rotation rotation, pros::Distance distance);
  ~Catapult();
  void fire();
  void arm();
  bool isArmed();
  bool isReady();
  int getShotNUm();

 private:
  pros::Motor motor;
  pros::Rotation rotation;
  pros::Distance distance;
  pros::Task internalTask;
  debouncer<double> debounce{0, 100, 100};
  void taskFunction();
  okapi::QAngle getAngle();
  std::atomic_bool shouldArm = false;
  std::atomic_bool shouldFire = false;
  std::atomic_bool armed = false;
  std::atomic_bool ready = false;
  int shotNum = 0;
};