#pragma once
#include "main.h"

enum class RollerColor {
    RED,
    BLUE
};

class Intake {
public:
  Intake(pros::MotorGroup imotorGroup, pros::Optical iopticalSensor, RollerColor color);
  ~Intake();
  void stop();
  void reverse();
  void forward();
  void waitUntilSettled(uint32_t timeoutMillis);
  bool isSettled();

protected:
  pros::Task internalTask;
  pros::MotorGroup motor;
  pros::Optical sensor;
};