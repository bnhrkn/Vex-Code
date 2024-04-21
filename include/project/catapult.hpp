#pragma once
#include <utility>

#include "main.h"
#include "project/PIDF.hpp"
#include "project/algorithms.hpp"

class Catapult {
 public:
  Catapult(pros::Motor motor, pros::Rotation rotation, pros::Distance distance);
  ~Catapult();
  // void fire();
  // void arm();
  void setArmMode(bool armMode,
                  uint32_t shotNums = std::numeric_limits<uint32_t>::max());
  bool getArmMode();
  bool isArmed();
  bool isReady();
  int getShotNum();

 private:
  pros::Motor motor;
  pros::Rotation rotation;
  pros::Distance distance;
  pros::Task internalTask;
  debouncer<double> debounce{0, 100, 100};
  void taskFunction();
  okapi::QAngle getAngle();
  // std::atomic_bool shouldArm = false;
  // std::atomic_bool shouldFire = false;
  bool alwaysArm = false;
  std::atomic_bool armed = false;
  std::atomic_bool ready = false;
  int shotNum = 0;
};