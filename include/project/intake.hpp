#pragma once
#include "main.h"

using hueRange = std::pair<int,int>;

class Intake {
public:
  enum class Mode;
  enum class State;
  Intake(pros::Motor motor, pros::Optical rollerSensor,
               pros::Distance indexerSensor, hueRange targetColor, hueRange otherColor, okapi::ControllerButton reverseBtn);
  ~Intake();
  void setManualMode(bool manual);
  void toggleManualMode();
  void setEnabledMode(bool enabled);
  void toggleEnabledMode();
  void flipRaw(okapi::QAngle amount);
  void waitUntilSettled(uint32_t timeoutMillis = TIMEOUT_MAX);
  bool isSettled();

protected:
  void taskFunction();
  std::atomic_bool manual = false;
  std::atomic_bool enabled = true;
  hueRange targetColor;
  hueRange otherColor;
  pros::Motor intakeMotor;
  pros::Optical rollerSensor;
  pros::Distance indexerSensor;
  okapi::ControllerButton reverseBtn;
  pros::Task internalTask;
};