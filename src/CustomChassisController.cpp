#include "project/CustomChassisController.hpp"
#include <cmath>
#include <utility>
#include "fmt/format.h"
#include "okapi/api/odometry/odomMath.hpp"
#include "project/algorithms.hpp"
using namespace okapi::literals;

CustomChassisController::CustomChassisController(
    std::shared_ptr<okapi::ChassisModel> imodel,
    std::shared_ptr<okapi::IterativePosPIDController> iturnPID,
    std::shared_ptr<okapi::IterativePosPIDController> idistancePID,
    std::shared_ptr<CustomOdom> iodom,
    okapi::ChassisScales ichassisScales,
    okapi::AbstractMotor::GearsetRatioPair idriveRatio)
    : movementTask([this]() { movementLoop(); }),
      odomTask([this]() { odomLoop(); }),
      turnPID(std::move(iturnPID)),
      distancePID(std::move(idistancePID)),
      odom(std::move(iodom)),
      model(std::move(imodel)),
      chassisScales(ichassisScales),
      driveRatio(idriveRatio) {
  turnPID->setTarget(okapi::OdomMath::constrainAngle360(odom->getState().theta)
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
  fmt::print("Destructor called\n");
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
    if (pros::Task::notify_take(true, 0) != 0) {
      valid = false;  // Invalidate the task when notified
    }
    return valid;
  };
  pros::Task::current().suspend();  // Wait to be resumed for first command
  fmt::print("Movement task awoke\n");
  while (taskValid()) {
    // End task when notified
    switch (mode.load()) {
      case MovementType::path: {
        fmt::print("Entered path case\n");
        std::scoped_lock<pros::Mutex> lock(movementMutex);
        auto prev = std::make_unique<std::uint32_t>(pros::millis());
        std::uint32_t timeDelta = (path[1].time - path[0].time) * 1000;
        for (auto& point : path) {
          if (!taskValid() || mode.load() != MovementType::path) {
            break;
          }
          // printf("Running path\n");
          // std::cout << "Command Position: " << point.vector.pose.x <<
          // std::endl;
          //  std::cout << "Actual Positin" << odom->getState().str(1_in,
          //  "") << std::endl;

          // Do all the pathy stuff
          const auto speeds =
              ramsete(stateToPose(odom->getState()), point, 2.0, 0.5);
          const auto [left, right] =
              chassisToTankSpeeds(speeds, chassisScales, driveRatio);
          // std::cout << "left: " << left.convert(1_rpm) << ", right: " <<
          // right.convert(1_rpm) << std::endl;
          model->left(left.convert(600_rpm));
          model->right(right.convert(600_rpm));
          pros::Task::delay_until(prev.get(), timeDelta);
        }
        fmt::print("Done path\n");
        model->stop();
        break;
      }

      case MovementType::turn: {
        fmt::print("Entered turning case\n");
        std::scoped_lock<pros::Mutex> lock(movementMutex);
        auto prev = std::make_unique<std::uint32_t>(pros::millis());
        // auto positiveModulo = [](double a, double b){
        //   return fmod(fmod())
        // }
        while (taskValid() && mode.load() == MovementType::turn) {
          // printf("Running turn\n");
          //   Do math to figure out the right direction to turn
          auto nowAngle =
              okapi::OdomMath::constrainAngle360(odom->getState().theta);
          auto error = okapi::OdomMath::constrainAngle180(
              turnPID->getTarget() * 1_deg - nowAngle);
          auto inputValue =
              turnPID->getTarget() -
              error.convert(1_deg);  // Fool the controller to use our error
          turnPID->step(inputValue);

          if (turnPID->isSettled()) {
            break;  // Must be after step or will instantly settle
          }

          // std::cout << "Now Angle: " << nowAngle.convert(1_deg) << ", Error:
          // "
          // << error.convert(1_deg)
          //           << ", Target: " << turnPID->getTarget()
          //           << ", Real Error: " << turnPID->getError() << std::endl;
          auto angleOutput = turnLimiter.filterIncrease(turnPID->getOutput());

          model->rotate(angleOutput);
          pros::Task::delay_until(prev.get(), 10);
        }
        fmt::print("Done Turn\n");
        model->stop();
        break;
      }
      case MovementType::straight: {
        fmt::print("Entered straight case\n");
        std::scoped_lock<pros::Mutex> lock(movementMutex);
        auto prev = std::make_unique<std::uint32_t>(pros::millis());
        auto initalPos = odom->getState();

        while (taskValid() && mode.load() == MovementType::straight) {
          auto nowTranslatedPos = translatePoint(odom->getState(), initalPos);
          auto nowRotatedPos =
              rotateAroundOrigin(nowTranslatedPos, -initalPos.theta);
          auto x = nowRotatedPos.x.convert(
              okapi::meter);  // Straight distance from where we started
          // std::cout << "InitalPos: " << initalPos.str()
          //           << "\nTransPos: " << nowTranslatedPos.str()
          //           << "\nrotatedPos: " << nowRotatedPos.str()
          //           << "\nTarget: " << distancePID->getTarget() << std::endl;
          distancePID->step(x);

          // Angle stuff to remain on course
          auto nowAngle =
              okapi::OdomMath::constrainAngle360(odom->getState().theta);
          auto angleError = okapi::OdomMath::constrainAngle180(
              turnPID->getTarget() * 1_deg - nowAngle);
          auto angleInputValue =
              turnPID->getTarget() -
              angleError.convert(
                  1_deg);  // Fool the controller to use our error
          turnPID->step(angleInputValue);

          if (distancePID->isSettled()) {
            break;
          }

          auto distanceOutput =
              distanceLimiter.filterIncrease(distancePID->getOutput());
          auto angleOutput = turnLimiter.filterIncrease(turnPID->getOutput());
          // model->driveVector(distancePID->getOutput(), turnPID->getOutput());
          model->forward(distanceOutput);
          pros::Task::delay_until(prev.get(), 10);
        }
        fmt::print("Done Straight\n");
        model->stop();
        break;
      }

      case MovementType::disabled:  // Do nothing
        fmt::print("Warning: Entered disabled case");
        break;
    }
    pros::Task::current().suspend();  // Suspend and await next command
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
  fmt::print("done all the stuff\n");
  // pros::delay(50);
}

void CustomChassisController::turnByAngle(okapi::QAngle angle) {
  turnToAngle(odom->getState().theta + angle);
}

void CustomChassisController::runPath(
    std::vector<squiggles::ProfilePoint> ipath) {
  fmt::print("Sending runpath command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  path = std::move(ipath);
  mode.store(MovementType::path);
  movementTask.resume();
}

// May need to update driveDistance for poss. of changed theta since prev
// movement

void CustomChassisController::driveDistance(okapi::QLength distance) {
  fmt::print("Sending moveDistance command\n");
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
            << odom->getPoint().x.convert(1_in) << ", "
            << odom->getPoint().y.convert(1_in) << ")\n";
  turnToAngle(angleToPoint(point, odom->getPoint()) + offset);
  waitUntilSettled();
}

void CustomChassisController::drivePathFromHere(
    std::vector<okapi::OdomState> points,
    squiggles::SplineGenerator& generator) {
  std::vector<squiggles::Pose> poses(points.size() + 1);
  poses.front() = stateToPose(odom->getState());
  std::ranges::transform(points, poses.begin() + 1, stateToPose);
  std::cout << "Generating path throstd::unique_ptr<std::unique_ptr<ugh ";
  for (auto pose : poses) {
    std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.yaw << "), ";
  }
  std::cout << "\n";
  auto path = generator.generate(poses);
  runPath(path);
}

void CustomChassisController::driveToPointByPath(
    okapi::OdomState destination,
    squiggles::SplineGenerator& generator,
    bool discreteTurn) {
  fmt::print("Driving to state {} from {}\n", destination.str(1_tile, "_t"),
             odom->getState().str(1_tile, "_t"));
  if (discreteTurn) {
    turnToPoint(stateToPoint(destination));
  }
  drivePathFromHere({{destination}}, generator);
}

void CustomChassisController::moveRaw(double speed,
                                      std::optional<std::uint32_t> time) {
  if (!isSettled()) {
    return;
  }

  model->forward(speed);

  if (time.has_value()) {
    pros::delay(time.value());
    cancelMovement();
  }
}

void CustomChassisController::turnRaw(double speed,
                                      std::optional<std::uint32_t> time) {
  if (!isSettled()) {
    return;
  }

  model->rotate(-speed);

  if (time.has_value()) {
    pros::delay(time.value());
    cancelMovement();
  }
}

void CustomChassisController::driveToPoint(
    okapi::Point point,
    bool reverse,
    okapi::QLength
        distanceThreshold) {  // TODO: when angle is closer to the back, reverse
  if (distanceToPoint(point, odom->getPoint()) < distanceThreshold) {
    return;
  }

  turnToPoint(point, static_cast<int>(reverse) * 180_deg);
  auto distance = distanceToPoint(point, odom->getPoint());
  fmt::print("Driving {}\" to point ({}\", {}\") from ({}\", {}\")\n",
             distance.convert(1_in), point.x.convert(1_in),
             point.y.convert(1_in), odom->getPoint().x.convert(1_in),
             odom->getPoint().y.convert(1_in));
  driveDistance(reverse ? -distance : distance);
  waitUntilSettled();
}

void CustomChassisController::cancelMovement() {
  fmt::print("Cancelling movement\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  waitUntilSettled();
  model->stop();
}