#include <cassert>
#include <utility>

#include "project/MPChassisController.hpp"
#include "project/algorithms.hpp"
#include "project/geometry.hpp"

using QUnitless = okapi::
    RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>>;

okapi::QAngularSpeed value(MotorColor color) {
  return static_cast<double>(color) * okapi::rpm;
}

okapi::AbstractMotor::GearsetRatioPair toGearset(MotorColor color,
                                                 double ratio) {
  switch (color) {
    case MotorColor::Red:
      return {okapi::AbstractMotor::gearset::red, ratio};
    case MotorColor::Green:
      return {okapi::AbstractMotor::gearset::green, ratio};
    case MotorColor::Blue:
      return {okapi::AbstractMotor::gearset::blue, ratio};
  }
}

MPChassisController::MPChassisController(
    std::shared_ptr<okapi::ChassisModel> model,
    std::shared_ptr<Tracker> tracker,
    DriveInfo driveInfo,
    Constraints constraints)
    : model(std::move(model)),
      tracker(std::move(tracker)),
      driveInfo(driveInfo),
      constraints(constraints),
      task([this]() { taskFunc(); }) {}

MPChassisController::~MPChassisController() {
  task.remove();
  model->stop();
}

void MPChassisController::taskFunc() {
  using namespace okapi::literals;
  const okapi::QSpeed maxSpeed = driveInfo.maxSpeed();
  // driveInfo.motorToVel(value(driveInfo.color) * driveInfo.ratio);
  const okapi::QAngularSpeed maxAngularSpeed = driveInfo.maxAngSpeed();
  // driveInfo.motorToAngVel(value(driveInfo.color) * driveInfo.ratio);
  while (true) {
    task.suspend();  // Wait for a movement
    // std::cout << "Movement loop awoke\n";
    // std::cout << "x,y,theta,linearVel,angularVel,linearAccel,angularAccel\n";
    auto startTime = pros::millis();
    for (auto& point : path) {
      std::cout << "Time: " << pros::millis() << "ms\n";
      auto speeds = ramsete(tracker->getState(), point);
      double linearCommand =
          (speeds.linearVel / maxSpeed).convert(QUnitless(1.0));
      double angularCommand =
          -(speeds.angularVel / maxAngularSpeed).convert(QUnitless(1.0));

      model->driveVector(linearCommand, angularCommand);

      // std::cout << std::format(
      //     "{},{},{},{},{},{},{}\n", point.pose.x.convert(1_m),
      //     point.pose.y.convert(1_m), point.pose.theta.convert(1_rad),
      //     point.linearVel.convert(1_mps), point.angularVel.convert(1_rad /
      //     1_s), point.linearAccel.convert(1_mps / 1_s),
      //     point.angularAccel.convert(1_rad / 1_s / 1_s));

      // std::cout << std::format(
      //     "Linear: {}, Angular: {}\n",
      //     (point.linearVel / maxSpeed).convert(QUnitless(1.0)),
      //     (point.angularVel / maxAngularSpeed).convert(QUnitless(1.0)));
      // model->driveVector(
      //     (point.linearVel / maxSpeed).convert(QUnitless(1.0)),
      //     (-point.angularVel / maxAngularSpeed).convert(QUnitless(1.0)));

      pros::Task::delay_until(&startTime, dt.convert(okapi::millisecond));
    }
    model->stop();
    // std::cout << "Movement loop finished\n";
  }
}

void MPChassisController::moveTo(okapi::OdomState target,
                                 okapi::QLength posTolerance,
                                 okapi::QAngle angleTolerance) {
  moveTo(stateToPoint(target), posTolerance, angleTolerance);
  if (geometry::angleDistance(tracker->getState().theta, target.theta) >
      angleTolerance) {
    // If the end state has different heading, turn
    path =
        genAngularProfile(tracker->getState(), target.theta, constraints, dt);
    task.resume();
    waitUntilSettled();
  }
}

void MPChassisController::moveTo(okapi::Point target,
                                 okapi::QLength posTolerance,
                                 okapi::QAngle angleTolerance) {
  auto origin = tracker->getState();
  auto angleToPoint = geometry::angleToPoint(target, stateToPoint(origin));
  if (geometry::distanceToPoint(target, stateToPoint(origin)) <= posTolerance) {
    // We are within the tolerance already
    return;
  }
  std::cout << std::format(
      "origin.theta {}, angleToPoint {}, geometry::angleDifference {}, "
      "angleTolerance {}\n",
      origin.theta.convert(okapi::radian), angleToPoint.convert(okapi::radian),
      geometry::angleDifference(origin.theta, angleToPoint)
          .convert(okapi::radian),
      angleTolerance.convert(okapi::radian));
  if (abs(geometry::angleDifference(origin.theta, angleToPoint)) >
      angleTolerance) {
    // If we need to turn first
    std::cout << "Running Angular Path\n";
    path = genAngularProfile(origin, angleToPoint, constraints, dt);
    task.resume();
    waitUntilSettled();
    std::cout << "Angular Path Finished, Task state: " << task.get_state()
              << "\n";
  }
  // Will only get here if we need a linear move
  origin = tracker->getState();
  path = genLinearProfile(stateToPoint(origin), target, constraints, dt);
  task.resume();
  waitUntilSettled();
}

void MPChassisController::moveBy(okapi::QLength distance) {
  okapi::OdomState origin = tracker->getState();
  okapi::Point target = {
      std::cos(origin.theta.convert(okapi::radian)) * distance,
      std::sin(origin.theta.convert(okapi::radian)) * distance};
  std::cout << std::format("moveBy sending target ({}_ft, {}_ft) to moveTo\n",
                           target.x.convert(okapi::foot),
                           target.y.convert(okapi::foot));
  moveTo(target, 0 * okapi::inch, 0.5 * okapi::degree);
}

void MPChassisController::turnTo(okapi::QAngle target,
                                 okapi::QAngle angleTolerance) {
  okapi::OdomState origin = tracker->getState();
  moveTo({origin.x, origin.y, target}, 10 * okapi::foot, angleTolerance);
}

void MPChassisController::turnBy(okapi::QAngle target) {
  turnTo(tracker->getState().theta + target, 0 * okapi::degree);
}

bool MPChassisController::isSettled() {
  // Task is not running, ready to run, or blocked then its settled
  return task.get_state() >= pros::E_TASK_STATE_SUSPENDED;
}
void MPChassisController::waitUntilSettled() {
  while (!isSettled()) {
    pros::delay(dt.convert(okapi::millisecond));
  }
}
void MPChassisController::stop() {
  task.suspend();
  model->stop();
}

[[nodiscard]] okapi::QLength DriveInfo::wheelCircumference() const {
  return wheelSize * okapi::pi;
}
[[nodiscard]] okapi::QSpeed DriveInfo::motorToVel(
    okapi::QAngularSpeed motorSpeed) const {
  return motorSpeed / okapi::radian * wheelSize / 2;
}
[[nodiscard]] okapi::QAngularSpeed DriveInfo::motorToAngVel(
    okapi::QAngularSpeed motorSpeed) const {
  return motorSpeed * wheelSize / trackWidth;
}
[[nodiscard]] okapi::QAngularSpeed DriveInfo::velToMotor(
    okapi::QSpeed vel) const {
  return vel * okapi::radian / wheelSize / 2 * 2;
}
[[nodiscard]] okapi::QAngularSpeed DriveInfo::angVelToMotor(
    okapi::QAngularSpeed angVel) const {
  return angVel * trackWidth / wheelSize;
}
[[nodiscard]] okapi::QSpeed DriveInfo::maxSpeed() const {
  return motorToVel(value(color) * ratio);
}
[[nodiscard]] okapi::QAngularSpeed DriveInfo::maxAngSpeed() const {
  return motorToAngVel(value(color) * ratio);
}
