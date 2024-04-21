#pragma once
#include "project/motionProfiling.hpp"
#include "project/tracker.hpp"

enum class MotorColor { Red = 100, Green = 200, Blue = 600 };

okapi::QAngularSpeed value(MotorColor color);

okapi::AbstractMotor::GearsetRatioPair toGearset(MotorColor color,
                                                 double ratio = 1.0);

struct DriveInfo {
  okapi::QLength wheelSize;
  okapi::QLength trackWidth;
  double ratio;
  MotorColor color;
  [[nodiscard]] okapi::QLength wheelCircumference() const;
  [[nodiscard]] okapi::QSpeed motorToVel(okapi::QAngularSpeed motorSpeed) const;
  [[nodiscard]] okapi::QAngularSpeed motorToAngVel(
      okapi::QAngularSpeed motorSpeed) const;
  [[nodiscard]] okapi::QAngularSpeed velToMotor(okapi::QSpeed vel) const;
  [[nodiscard]] okapi::QAngularSpeed angVelToMotor(
      okapi::QAngularSpeed angVel) const;
  [[nodiscard]] okapi::QSpeed maxSpeed() const;
  [[nodiscard]] okapi::QAngularSpeed maxAngSpeed() const;
};

class MPChassisController {
 public:
  MPChassisController(std::shared_ptr<okapi::ChassisModel> model,
                      std::shared_ptr<Tracker> tracker,
                      DriveInfo driveInfo,
                      Constraints constraints);
  ~MPChassisController();
  void moveTo(okapi::OdomState target,
              okapi::QLength posTolerance = okapi::inch,
              okapi::QAngle angleTolerance = 0.5 * okapi::degree);
  void moveTo(okapi::Point target,
              okapi::QLength posTolerance = okapi::inch,
              okapi::QAngle angleTolerance = 0.5 * okapi::degree);
  void moveBy(okapi::QLength distance);
  void turnTo(okapi::QAngle target,
              okapi::QAngle angleTolerance = 0.5 * okapi::degree);
  void turnBy(okapi::QAngle target);
  void stop();

  bool isSettled();
  void waitUntilSettled();

 private:
  std::shared_ptr<okapi::ChassisModel> model;
  std::shared_ptr<Tracker> tracker;
  DriveInfo driveInfo;
  Constraints constraints;
  pros::Task task;
  std::vector<ProfilePoint> path = {};
  okapi::QTime dt = 10 * okapi::millisecond;
  void taskFunc();
};