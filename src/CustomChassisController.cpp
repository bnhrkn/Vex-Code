#include "project/CustomChassisController.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "project/algorithms.hpp"
#include <cmath>
using namespace okapi::literals;

CustomChassisController::CustomChassisController(
    std::shared_ptr<okapi::ChassisModel> imodel,
    std::shared_ptr<okapi::IterativePosPIDController> iturnPID,
    std::shared_ptr<okapi::IterativePosPIDController> idistancePID,
    std::shared_ptr<okapi::Odometry> iodom, okapi::ChassisScales ichassisScales,
    okapi::AbstractMotor::GearsetRatioPair idriveRatio)
    : movementTask([this]() { movementLoop(); }),
      odomTask([this]() { odomLoop(); }), turnPID(iturnPID),
      distancePID(idistancePID), odom(iodom), model(imodel),
      chassisScales(ichassisScales), driveRatio(idriveRatio) {
  turnPID->setTarget(
      okapi::OdomMath::constrainAngle360(getConvertedState(odom).theta)
          .convert(1_deg));
  distancePID->setTarget(0);
  // pros::c::task_notify_when_deleting(
  //     pros::c::task_get_current(), (pros::task_t)movementTask, 1,
  //     pros::notify_action_e_t::E_NOTIFY_ACTION_OWRITE);
  // pros::c::task_notify_when_deleting(
  //     pros::c::task_get_current(), (pros::task_t)odomTask, 1,
  //     pros::notify_action_e_t::E_NOTIFY_ACTION_OWRITE);
  waitUntilSettled();
}

CustomChassisController::~CustomChassisController() {
  printf("Destructor called\n");
  movementTask.notify();
  pros::c::task_abort_delay((pros::task_t)movementTask);
  odomTask.notify();
  pros::c::task_abort_delay((pros::task_t)odomTask);
  movementTask.join();
  odomTask.join();
  model->stop();
}

void CustomChassisController::movementLoop() {
  auto taskValid = []() {
    static bool valid = true;
    if (pros::Task::notify_take(true, 0) != 0)
      valid = false; // Invalidate the task when notified
    return valid;
  };
  pros::Task::current().suspend(); // Wait to be resumed for first command
  printf("Movement task awoke\n");
  while (taskValid()) {
    // End task when notified
    switch (mode.load()) {

    case MovementType::path: {
      printf("Entered path case\n");
      std::scoped_lock<pros::Mutex> lock(movementMutex);
      auto prev = std::make_unique<std::uint32_t>(pros::millis());
      auto timeDelta = (path[1].time - path[0].time) * 1000;
      for (auto &point : path) {
        if (!taskValid() || mode.load() != MovementType::path)
          break;
        // printf("Running path\n");
        // std::cout << "Command Position: " << point.vector.pose.x <<
        // std::endl;
        //  std::cout << "Actual Positin" << getConvertedState(odom).str(1_in,
        //  "") << std::endl;

        // Do all the pathy stuff
        const auto speeds =
            ramsete(stateToPose(getConvertedState(odom)), point, 2.0, 0.7);
        const auto [left, right] =
            chassisToTankSpeeds(speeds, chassisScales, driveRatio);
        // std::cout << "left: " << left.convert(1_rpm) << ", right: " <<
        // right.convert(1_rpm) << std::endl;
        model->left(left.convert(600_rpm));
        model->right(right.convert(600_rpm));
        pros::Task::delay_until(prev.get(), timeDelta);
      }
      printf("Done path\n");
      model->stop();
      break;
    }

    case MovementType::turn: {
      printf("Entered turning case\n");
      std::scoped_lock<pros::Mutex> lock(movementMutex);
      auto prev = std::make_unique<std::uint32_t>(pros::millis());
      // auto positiveModulo = [](double a, double b){
      //   return fmod(fmod())
      // }
      while (taskValid() && mode.load() == MovementType::turn) {
        // printf("Running turn\n");
        //   Do math to figure out the right direction to turn
        auto nowAngle =
            okapi::OdomMath::constrainAngle360(getConvertedState(odom).theta);
        auto error = okapi::OdomMath::constrainAngle180(
            turnPID->getTarget() * 1_deg - nowAngle);
        auto inputValue =
            turnPID->getTarget() -
            error.convert(1_deg); // Fool the controller to use our error
        turnPID->step(inputValue);

        if (turnPID->isSettled())
          break; // Must be after step or will instantly settle

        // std::cout << "Now Angle: " << nowAngle.convert(1_deg) << ", Error: "
        // << error.convert(1_deg)
        //           << ", Target: " << turnPID->getTarget()
        //           << ", Real Error: " << turnPID->getError() << std::endl;
        auto angleOutput = turnLimiter.filterIncrease(turnPID->getOutput());

        model->rotate(angleOutput);
        pros::Task::delay_until(prev.get(), 10);
      }
      printf("Done Turn\n");
      model->stop();
      break;
    }
    case MovementType::straight: {
      printf("Entered straight case\n");
      std::scoped_lock<pros::Mutex> lock(movementMutex);
      auto prev = std::make_unique<std::uint32_t>(pros::millis());
      auto initalPos = getConvertedState(odom);

      while (taskValid() && mode.load() == MovementType::straight) {
        auto nowTranslatedPos =
            translatePoint(getConvertedState(odom), initalPos);
        auto nowRotatedPos =
            rotateAroundOrigin(nowTranslatedPos, -initalPos.theta);
        auto x = nowRotatedPos.x.convert(
            okapi::meter); // Straight distance from where we started
        // std::cout << "InitalPos: " << initalPos.str()
        //           << "\nTransPos: " << nowTranslatedPos.str()
        //           << "\nrotatedPos: " << nowRotatedPos.str()
        //           << "\nTarget: " << distancePID->getTarget() << std::endl;
        distancePID->step(x);

        // Angle stuff to remain on course
        auto nowAngle =
            okapi::OdomMath::constrainAngle360(getConvertedState(odom).theta);
        auto angleError = okapi::OdomMath::constrainAngle180(
            turnPID->getTarget() * 1_deg - nowAngle);
        auto angleInputValue =
            turnPID->getTarget() -
            angleError.convert(1_deg); // Fool the controller to use our error
        turnPID->step(angleInputValue);

        if (distancePID->isSettled())
          break;

        auto distanceOutput =
            distanceLimiter.filterIncrease(distancePID->getOutput());
        auto angleOutput = turnLimiter.filterIncrease(turnPID->getOutput());
        // model->driveVector(distancePID->getOutput(), turnPID->getOutput());
        model->forward(distanceOutput);
        pros::Task::delay_until(prev.get(), 10);
      }
      printf("Done Straight\n");
      model->stop();
      break;
    }

    case MovementType::disabled: // Do nothing
      printf("Warning: Entered disabled case");
      break;
    }
    pros::Task::current().suspend(); // Suspend and await next command
  }
}

void CustomChassisController::odomLoop() {
  auto prev = std::make_unique<std::uint32_t>(pros::millis());
  while (pros::Task::notify_take(true, 0) == 0) {
    odom->step();
    pros::Task::delay_until(prev.get(), 10);
  }
}

bool CustomChassisController::isSettled() {
  return movementTask.get_state() == pros::E_TASK_STATE_SUSPENDED;
}

void CustomChassisController::waitUntilSettled() {
  while (!isSettled()) {
    pros::delay(10);
  }
}

void CustomChassisController::turnToAngle(okapi::QAngle angle) {
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  turnPID->reset();
  turnPID->setTarget(okapi::OdomMath::constrainAngle360(angle).convert(1_deg));
  std::cout << "Turning to angle " << angle.convert(1_deg) << "deg by target "
            << okapi::OdomMath::constrainAngle360(angle).convert(1_deg)
            << "deg\n";
  mode.store(MovementType::turn);
  movementTask.resume();
  printf("done all the stuff\n");
  // pros::delay(50);
}

void CustomChassisController::turnByAngle(okapi::QAngle angle) {
  turnToAngle(getConvertedState(odom).theta + angle);
}

void CustomChassisController::runPath(
    std::vector<squiggles::ProfilePoint> ipath) {
  printf("Sending runpath command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  path = std::move(ipath);
  mode.store(MovementType::path);
  movementTask.resume();
}

void CustomChassisController::driveDistance(okapi::QLength distance) {
  printf("Sending moveDistance command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  distancePID->reset();
  distancePID->setTarget(distance.convert(okapi::meter));
  std::cout << "Driving distance " << distance.convert(1_in) << "in\n";
  // Angle target remains the same
  mode.store(MovementType::straight);
  movementTask.resume();
}

void CustomChassisController::turnToPoint(okapi::Point point,
                                          okapi::QAngle offset) {
  std::cout << "Turning to point (" << point.x.convert(1_in) << ", "
            << point.y.convert(1_in) << ") from ("
            << getConvertedPoint(odom).x.convert(1_in) << ", "
            << getConvertedPoint(odom).y.convert(1_in) << ")\n";
  turnToAngle(angleToPoint(point, getConvertedPoint(odom)) + offset);
  waitUntilSettled();
}

void CustomChassisController::moveRaw(double speed,
                                      std::optional<std::uint32_t> time) {
  if (!isSettled())
    return;

  model->forward(speed);

  if (time.has_value()) {
    pros::delay(time.value());
    cancelMovement();
  }
}

void CustomChassisController::turnRaw(double speed,
                                      std::optional<std::uint32_t> time) {
  if (!isSettled())
    return;

  model->rotate(-speed);

  if (time.has_value()) {
    pros::delay(time.value());
    cancelMovement();
  }
}

void CustomChassisController::driveToPoint(
    okapi::Point point, bool reverse,
    okapi::QLength
        distanceThreshold) { // TODO: when angle is closer to the back, reverse
  if (distanceToPoint(point, getConvertedPoint(odom)) < distanceThreshold)
    return;

  turnToPoint(point, static_cast<int>(reverse) * 180_deg);
  auto distance = distanceToPoint(point, getConvertedPoint(odom));
  std::cout << "Driving " << distance.convert(1_in) << "in to point ("
            << point.x.convert(1_in) << ", " << point.y.convert(1_in)
            << ") from (" << getConvertedPoint(odom).x.convert(1_in) << ", "
            << getConvertedPoint(odom).y.convert(1_in) << ")\n";
  driveDistance(reverse ? -distance : distance);
  waitUntilSettled();
}

void CustomChassisController::cancelMovement() {
  printf("Cancelling movement\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  waitUntilSettled();
  model->stop();
}