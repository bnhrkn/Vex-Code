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
         pros::Optical optical,
         pros::Vision vision,
         pros::adi::DigitalIn liftSwitch);
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
  pros::Optical optical;
  pros::Vision vision;
  pros::adi::DigitalIn liftSwitch;
  pros::Task internalTask;
  pros::vision_signature green_sig =
      pros::c::vision_signature_from_utility(1,
                                             -4097,
                                             -1985,
                                             -3042,
                                             -3971,
                                             -1871,
                                             -2920,
                                             1.500,
                                             0);
};