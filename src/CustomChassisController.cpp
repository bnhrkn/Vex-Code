#include "project/CustomChassisController.hpp"
#include "project/ramsete.hpp"
#include <cmath>
using namespace okapi::literals;

CustomChassisController::CustomChassisController(
    std::shared_ptr<okapi::ChassisModel> imodel,
    std::shared_ptr<okapi::IterativePosPIDController> iturnPID,
    std::shared_ptr<okapi::Odometry> iodom, okapi::ChassisScales ichassisScales,
    okapi::AbstractMotor::GearsetRatioPair idriveRatio)
    : movementTask([this]() { movementLoop(); }),
      odomTask([this]() { odomLoop(); }), turnPID(iturnPID), odom(iodom),
      model(imodel), chassisScales(ichassisScales), driveRatio(idriveRatio) {
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
  movementTask.remove();
  odomTask.remove();
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

    case MovementType::straight: {
      printf("Entered straight path case\n");
      std::scoped_lock<pros::Mutex> lock(movementMutex);
      auto prev = std::make_unique<std::uint32_t>(pros::millis());
      auto timeDelta = (path[1].time - path[0].time) * 1000;
      for (auto &point : path) {
        if (!taskValid() || mode.load() != MovementType::straight)
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
        // std::cout << "left: " << left.convert(1_rpm) << ", right: " << right.convert(1_rpm) << std::endl;
        model->left(left.convert(600_rpm));
        model->right(right.convert(600_rpm));
        pros::Task::delay_until(prev.get(), timeDelta);
      }
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
        auto nowAngle = okapi::OdomMath::constrainAngle360(getConvertedState(odom).theta);
        auto error = okapi::OdomMath::constrainAngle180(turnPID->getTarget() * 1_deg - nowAngle);
        auto inputValue = turnPID->getTarget() -
                          error.convert(1_deg); // Fool the controller to use our error
        turnPID->step(inputValue);

        if(turnPID->isSettled()) break; // Must be after step or will instantly settle

        // std::cout << "Now Angle: " << nowAngle.convert(1_deg) << ", Error: " << error.convert(1_deg)
        //           << ", Target: " << turnPID->getTarget()
        //           << ", Real Error: " << turnPID->getError() << std::endl;

        model->rotate(turnPID->getOutput());
        pros::Task::delay_until(prev.get(), 10);
      }
      printf("Done Turn\n");
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
  printf("Sending turntoangle command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  turnPID->setTarget(okapi::OdomMath::constrainAngle360(angle).convert(1_deg));
  mode.store(MovementType::turn);
  movementTask.resume();
  printf("done all the stuff\n");
  //pros::delay(50);
}

void CustomChassisController::runPath(
    std::vector<squiggles::ProfilePoint> ipath) {
  printf("Sending runpath command\n");
  mode.store(MovementType::disabled);
  std::scoped_lock<pros::Mutex> lock(movementMutex);
  path = std::move(ipath);
  mode.store(MovementType::straight);
  movementTask.resume();
}