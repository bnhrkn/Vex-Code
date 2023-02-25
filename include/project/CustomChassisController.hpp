#pragma once
#include "main.h"
#include "project/algorithms.hpp"

class CustomChassisController {
public:
  CustomChassisController(
      std::shared_ptr<okapi::ChassisModel> imodel,
      std::shared_ptr<okapi::IterativePosPIDController> iturnPID,
      std::shared_ptr<okapi::IterativePosPIDController> idistancePID,
      std::shared_ptr<okapi::Odometry> iodom,
      okapi::ChassisScales ichassisScales,
      okapi::AbstractMotor::GearsetRatioPair idriveRatio);
  ~CustomChassisController();

  
  void runPath(std::vector<squiggles::ProfilePoint>); // Pass a fully formed path to follow

  void turnToAngle(okapi::QAngle angle);

  void turnToPoint(okapi::Point point, okapi::QAngle offset = 0 * okapi::degree);

  void driveDistance(okapi::QLength distance);

  void driveToPoint(okapi::Point point, bool reverse = false, okapi::QLength distanceThreshold = 0 * okapi::inch);

  // Delegate to the model, speed is [-1, 1], no delay or stop if no time given
  // Will not run over closed-loop movements
  void moveRaw(double speed, std::optional<std::uint32_t> time = std::nullopt);

  // Delegate to the model, speed is [-1, 1], no delay or stop if no time given
  // Will not run over closed-loop movements
  void turnRaw(double speed, std::optional<std::uint32_t> time = std::nullopt);

  // If the movementTask is done
  bool isSettled();

  // Wait for isSettled
  void waitUntilSettled();

  // Stop any open or closed-loop movements
  void cancelMovement();

protected:
  enum class MovementType { disabled = 0, path = 1, turn = 2,  straight = 3};
  std::atomic<MovementType> mode = MovementType::disabled;

  void movementLoop();
  void odomLoop();

  pros::Task movementTask;
  pros::Task odomTask;

  std::shared_ptr<okapi::IterativePosPIDController> turnPID;
  changeLimiter<double> turnLimiter{1,0};

  std::shared_ptr<okapi::IterativePosPIDController> distancePID;
  changeLimiter<double> distanceLimiter{1, 0};

  std::shared_ptr<okapi::Odometry> odom;

  std::shared_ptr<okapi::ChassisModel> model;

  std::vector<squiggles::ProfilePoint> path;

  pros::Mutex movementMutex; // Prevent overwriting current movement targets with new ones

  okapi::ChassisScales chassisScales;
  okapi::AbstractMotor::GearsetRatioPair driveRatio;
};