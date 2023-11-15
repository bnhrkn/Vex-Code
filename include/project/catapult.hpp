#pragma once
#include <utility>

#include "main.h"
#include "project/PIDF.hpp"

class Catapult {
 private:
  enum class CATAPULT_STATE {
    armed = 2,
    shooting = 3,
    idle = 0,
    loading = 1,
    jam = 4,
    unknown = 5,
  };

 public:
  Catapult(std::vector<std::int8_t> motorPorts,
           pros::Rotation rotation,
           pidf gains);
  ~Catapult();
  bool fire();
  bool isArmed();
  void setGains(pidf gains);
  pidf getGains();
  Catapult::CATAPULT_STATE getState();

 private:
  pidfAlgorithm controller;
  pros::MotorGroup motors;
  pros::Rotation rotation;
  pros::Task motorTask;
  void motorLoop();
  std::atomic<CATAPULT_STATE> state = CATAPULT_STATE::idle;
};