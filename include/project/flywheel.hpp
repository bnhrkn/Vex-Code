#pragma once
#include <atomic>
#include "main.h"
#include "project/settledUtil.hpp"

class Flywheel {
  using Gains = okapi::IterativeVelPIDController::Gains;

 public:
  Flywheel(pros::Motor motor, Gains gains, SettledUtil settledUtil);
  ~Flywheel();
  void setGains(Gains values);
  Gains getGains() const;
  void setTarget(okapi::QAngularSpeed speed);
  void setTarget(double rpm);
  okapi::QAngularSpeed getTarget() const;
  bool isSettled() const;
  void waitUntilSettled(const std::uint32_t pollingRate = 10) const;

 private:
  void taskFunc();
  pros::Motor motor;
  mutable pros::Mutex settledUtilMutex;
  SettledUtil settledUtil;
  pros::Task internalTask;
  std::atomic<double> target;
  std::atomic_uint32_t prevTime;
  mutable pros::Mutex gainsMutex;
  Gains gains;
};