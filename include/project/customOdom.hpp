#pragma once
#include "main.h"

class CustomOdom {
public:
  CustomOdom(std::shared_ptr<pros::Rotation> left,
             std::shared_ptr<pros::Rotation> right,
             std::shared_ptr<pros::IMU> imu, okapi::ChassisScales odomScales);
  void step();
  okapi::OdomState getState() const;
  okapi::Point getPoint() const;
  void setState(const okapi::OdomState &istate);

protected:
  struct SensorValues {
    SensorValues operator-(SensorValues values) const;
    okapi::QAngle x;
    //okapi::QAngle y;
    okapi::QAngle angle;
  };
  std::shared_ptr<pros::Rotation> left;
  std::shared_ptr<pros::Rotation> right;
  std::shared_ptr<pros::IMU> imu;
  SensorValues prevReading;

  const okapi::ChassisScales scales;

  mutable pros::Mutex stateMutex;
  okapi::OdomState state{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
};