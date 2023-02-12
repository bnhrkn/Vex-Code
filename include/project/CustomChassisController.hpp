#pragma once
#include "main.h"

class CustomChassisController {
public:
  CustomChassisController(
      std::shared_ptr<okapi::ChassisModel> imodel,
      std::shared_ptr<okapi::IterativePosPIDController> iturnPID,
      std::shared_ptr<okapi::Odometry> iodom,
      okapi::ChassisScales ichassisScales,
      okapi::AbstractMotor::GearsetRatioPair idriveRatio);
  ~CustomChassisController();

  
  void runPath(std::vector<squiggles::ProfilePoint>); // Pass a fully formed path to follow

  void turnToAngle(okapi::QAngle angle);

  bool isSettled();

  void waitUntilSettled();

protected:
  enum class MovementType { disabled = 0, straight = 1, turn = 2 };
  std::atomic<MovementType> mode = MovementType::disabled;

  void movementLoop();
  void odomLoop();

  pros::Task movementTask;
  pros::Task odomTask;

  std::shared_ptr<okapi::IterativePosPIDController> turnPID;

  std::shared_ptr<okapi::Odometry> odom;

  std::shared_ptr<okapi::ChassisModel> model;

  std::vector<squiggles::ProfilePoint> path;

  pros::Mutex movementMutex; // Prevent overwriting current movement targets with new ones

  okapi::ChassisScales chassisScales;
  okapi::AbstractMotor::GearsetRatioPair driveRatio;
};