#pragma once
#include "main.h"
#include "project/catapult.hpp"
#include "settledUtil.hpp"

using hueRange = std::pair<int, int>;
// test
class Intake {
 public:
  enum class Mode;
  enum class State;
  Intake(pros::Motor motor, pros::Optical sensor);
  ~Intake();
  void setManualMode(bool manual, int32_t voltage = 12000);
  void waitUntilSettled(uint32_t timeoutMillis = TIMEOUT_MAX);
  bool isSettled();
  bool hasBall();
  bool seeBall();
  void release();

 private:
  void taskFunction();
  std::atomic_bool manual = false;
  pros::Motor motor;
  pros::Optical sensor;
  pros::Task internalTask;
};