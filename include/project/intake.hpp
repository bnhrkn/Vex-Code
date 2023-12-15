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
  Intake(pros::Motor motor,
         std::shared_ptr<Catapult> cata,
         std::shared_ptr<okapi::SkidSteerModel> driveModel,
         pros::Optical outerSensor,
         pros::Optical innerSensor);
  ~Intake();
  void setManualMode(bool manual, int32_t voltage = 12000);
  // void toggleManualMode();
  //  void setLoopSkip(bool shouldSkip);
  //  void setEnabledMode(bool enabled);
  //  void toggleEnabledMode();
  //   Blocks until the motion is complete
  void waitUntilSettled(uint32_t timeoutMillis = TIMEOUT_MAX);
  bool isSettled();
  bool hasBall();
  bool seeBall();
  void release();

 private:
  void taskFunction();
  std::atomic_bool manual = false;
  //   std::atomic_bool enabled = true;
  //   std::atomic_bool shouldSkip = false;
  pros::Motor motor;
  pros::Optical outerSensor;
  pros::Optical innerSensor;
  pros::Task internalTask;
  std::shared_ptr<Catapult> cata;
  std::shared_ptr<okapi::SkidSteerModel> model;
};