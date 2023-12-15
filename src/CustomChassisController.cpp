#include "project/CustomChassisController.hpp"
#include <cmath>
#include <format>
#include <utility>
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
  std::cout << std::format("Destructor called\n");
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
  std::cout << std::format("Movement task awoke\n");
  while (taskValid()) {
    // End task when notified
    switch (mode.load()) {
      case MovementType::path: {
        std::cout << std::format("Entered path case\n");
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
          model->left(left.convert(200_rpm));
          model->right(right.convert(200_rpm));
          pros::Task::delay_until(prev.get(), timeDelta);
        }
        std::cout << std::format("Done path\n");
        model->stop();
        break;
      }

      case MovementType::turn: {
        std::cout << std::format("Entered turning case\n");
        std::scoped_lock<pros::Mutex> lock(movementMutex);
        auto prev = std::make_unique<std::uint32_t>(pros::millis());
        // auto positiveModulo = [](double a, double b){
        //   return fmod(fmod())
        // }
        while (taskValid() && mode.load() == MovementType::turn) {
          // printf("Running turn\n");
          //   Do math to figure out the right direction to turn
          auto nowAngle = odom->getState().theta;
          // okapi::OdomMath::constrainAngle360(odom->getState().theta);
          // auto error =
          //     turnPID->getTarget() * 1_deg - nowAngle;
          // auto inputValue =
          //     turnPID->getTarget() -
          //     error.convert(1_deg);  // Fool the controller to use our error
          turnPID->step(nowAngle.convert(1_deg));

          if (turnPID->isSettled()) {
            break;  // Must be after step or will instantly settle
          }

          // std::cout << "Now Angle: "
          //           << nowAngle.convert(1_deg)
          //           // << ", Error:" << error.convert(1_deg)
          //           << ", Target: " << turnPID->getTarget()
          //           << ", Real Error: " << turnPID->getError() << std::endl;
          auto angleOutput = turnLimiter.filterIncrease(turnPID->getOutput());

          model->rotate(-angleOutput);
          pros::Task::delay_until(prev.get(), 10);
        }
        std::cout << std::format("Done Turn\n");
        model->stop();
        break;
      }
      case MovementType::straight: {
        std::cout << std::format("Entered straight case\n");
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

          auto distanceOutput = std::clamp(
              distanceLimiter.filterIncrease(distancePID->getOutput()),
              -maxSpeed, maxSpeed);
          auto angleOutput = turnLimiter.filterIncrease(turnPID->getOutput());
          // model->driveVector(distancePID->getOutput(), turnPID->getOutput());
          model->forward(distanceOutput);
          // std::cout << std::format(
          //     "Ran Loop. Dist PID Input: {}, Angle PID Input {}\n",
          //     nowRotatedPos.x.convert(1_in),
          //     nowRotatedPos.theta.convert(1_deg));
          pros::Task::delay_until(prev.get(), 10);
        }
        std::cout << std::format("Done Straight\n");
        model->stop();
        break;
      }

      case MovementType::disabled:  // Do nothing
        std::cout << std::format("Warning: Entered disabled case");
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

  double target = angle.convert(1_deg);
  double theta = odom->getState().theta.convert(1_deg);
  // Normalize target angle
  target = fmod(target, 360.0);
  if (target < 0) {
    target += 360.0;
  }
  // Calculate and add the required number of full rotations
  double rotationCount = round((theta - target) / 360.0);
  double inputValue = target + rotationCount * 360.0;

  turnPID->setTarget(inputValue);
  std::cout << "Turning to angle " << angle.convert(1_deg) << "deg by target "
            << inputValue << "deg\n";
  mode.store(MovementType::turn);
  movementTask.resume();
  std::cout << std::format("done all the stuff\n");
  // pros::delay(50);
}

void CustomChassisController::turnByAngle(okapi::QAngle angle) {
  // turnToAngle(odom->getState().theta + angle);
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  turnPID->reset();
  turnPID->setTarget((odom->getState().theta + angle).convert(1_deg));
  std::cout << "Turning by angle " << angle.convert(1_deg) << "deg by target "
            << (odom->getState().theta + angle).convert(1_deg) << "deg\n";

  mode.store(MovementType::turn);
  movementTask.resume();
  std::cout << std::format("done all the stuff\n");
}

void CustomChassisController::runPath(
    std::vector<squiggles::ProfilePoint> ipath) {
  std::cout << std::format("Sending runpath command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  path = std::move(ipath);
  mode.store(MovementType::path);
  movementTask.resume();
}

// May need to update driveDistance for poss. of changed theta since prev
// movement

void CustomChassisController::driveDistance(okapi::QLength distance,
                                            double maxSpeed) {
  std::cout << std::format("Sending moveDistance command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  CustomChassisController::maxSpeed = maxSpeed;
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
  std::cout << std::format("Driving to state {} from {}\n",
                           destination.str(1_tile, "_t"),
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
    okapi::QLength distanceThreshold,
    double maxSpeed) {  // TODO: when angle is closer to the back, reverse
  if (distanceToPoint(point, odom->getPoint()) < distanceThreshold) {
    std::cout << std::format("Distance threshold not met: no movement");
    return;
  }

  turnToPoint(point, static_cast<int>(reverse) * 180_deg);
  auto distance = distanceToPoint(point, odom->getPoint());
  std::cout << std::format(
      "Driving {}\" to point ({}\", {}\") from ({}\", {}\")\n",
      distance.convert(1_in), point.x.convert(1_in), point.y.convert(1_in),
      odom->getPoint().x.convert(1_in), odom->getPoint().y.convert(1_in));
  driveDistance(reverse ? -distance : distance, maxSpeed);
  waitUntilSettled();
}

void CustomChassisController::driveToPoint(okapi::Point point,
                                           double maxSpeed) {
  driveToPoint(point, false, 0_in, maxSpeed);
}

void CustomChassisController::cancelMovement() {
  std::cout << std::format("Cancelling movement\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  waitUntilSettled();
  model->stop();
}