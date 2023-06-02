#include "main.h"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include "project/CustomChassisController.hpp"
#include "project/algorithms.hpp"
#include "project/flywheel.hpp"
#include "project/intake.hpp"
#include "project/pidTuner.hpp"
#include "project/settledUtil.hpp"
#include "project/ui.hpp"
#include "pros/rtos.hpp"

using namespace okapi::literals;

okapi::Controller controller;

const auto ratio = okapi::AbstractMotor::GearsetRatioPair{
    okapi::AbstractMotor::gearset::blue,
    36.0 / 60.0};  // motor rpm to wheel rpm
const auto constraints = squiggles::Constraints(2, 2, 9);
const auto slowConstraints = squiggles::Constraints(0.5, 2, 9);
const auto odomScales =
    okapi::ChassisScales({2.75_in, 7.46653976345_in, 4.6_in, 2.75_in}, 360);
const auto chassisScales =
    okapi::ChassisScales({3.25_in, 12.75_in}, okapi::imev5BlueTPR);

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

auto angleGains = okapi::IterativePosPIDController::Gains{0.00527, 0.000, 4e-6};
auto distanceGains =
    okapi::IterativePosPIDController::Gains{1.785, 0.0, 0.0001};

auto flywheelGains =
    okapi::IterativeVelPIDController::Gains{400.0, 0.0, 17.4, 864.0};

std::shared_ptr<okapi::ChassisModel> model;
std::shared_ptr<Flywheel> flywheel;
std::shared_ptr<CustomOdom> odometry;
std::shared_ptr<Intake> intake;
std::shared_ptr<CustomChassisController> chassis;
std::shared_ptr<okapi::IterativePosPIDController> turnPID;
std::shared_ptr<okapi::IterativePosPIDController> distancePID;

void on_center_button() {}

void initialize() {
  auto makeMotor = [](int iport, bool ireversed) {
    return okapi::Motor(iport, ireversed, ratio.internalGearset,
                        okapi::AbstractMotor::encoderUnits::counts);
  };

  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      "/ser/sout",                   // Output to the PROS terminal
      okapi::Logger::LogLevel::warn  // Show errors and warnings
      ));

  std::pair<int, int> blueRange = {220, 240};
  decltype(blueRange) redRange = {0, 25};

  auto hueRange = display.isBlueTeam() ? blueRange : redRange;
  auto wrongHueRange = display.isBlueTeam() ? redRange : blueRange;

  intake = std::make_shared<Intake>(
      pros::Motor(-4), pros::Optical(16), pros::Distance(13), hueRange,
      wrongHueRange, okapi::ControllerButton(okapi::ControllerDigital::left));

  flywheel = std::make_shared<Flywheel>(
      pros::Motor(8, pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE), flywheelGains,
      SettledUtil(1, 50, 0));

  std::ranges::for_each(std::array{14, 7}, pros::c::rotation_reset_position);

  model = std::make_shared<okapi::SkidSteerModel>(
      std::make_shared<okapi::MotorGroup>(okapi::MotorGroup{
          makeMotor(1, false), makeMotor(2, true), makeMotor(3, false)}),
      std::make_shared<okapi::MotorGroup>(okapi::MotorGroup{
          makeMotor(11, true), makeMotor(12, false), makeMotor(18, true)}),
      std::make_shared<okapi::IntegratedEncoder>(1, false),
      std::make_shared<okapi::IntegratedEncoder>(11, true), 600, 12000);

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  auto leftRotation = std::make_shared<pros::Rotation>(14, true);
  auto rightRotation = std::make_shared<pros::Rotation>(7, false);
  auto imu = std::make_shared<pros::IMU>(6);
  imu->reset(true);

  odometry = std::make_shared<CustomOdom>(leftRotation, rightRotation, imu,
                                          odomScales);

  odometry->setState({0_in, 0_in, 0_deg});

  turnPID = std::make_shared<okapi::IterativePosPIDController>(
      angleGains,
      okapi::ConfigurableTimeUtilFactory(0.75, 0.5, 100_ms).create(),
      // std::make_unique<lowPassFilter>(20, 10)
      std::make_unique<okapi::AverageFilter<3>>());

  distancePID = std::make_shared<okapi::IterativePosPIDController>(
      distanceGains,
      okapi::ConfigurableTimeUtilFactory(0.01, 0.02, 100_ms).create(),
      // std::make_unique<lowPassFilter>(20, 10)
      std::make_unique<okapi::AverageFilter<3>>());

  chassis = std::make_shared<CustomChassisController>(
      model, turnPID, distancePID, odometry, chassisScales, ratio);

  pros::delay(
      500);  // There is a race condition somewhere, sensors fail to reset.
}

void disabled() {
  chassis->cancelMovement();
  model->stop();
}

void competition_initialize() {}

void autonomous() {
  auto cylinder = pros::ADIDigitalOut(1, false);

  auto fastGen = squiggles::SplineGenerator(
      constraints,
      std::make_shared<squiggles::TankModel>(
          chassisScales.wheelTrack.convert(1_m), constraints),
      0.01);

  auto slowGen = squiggles::SplineGenerator(
      slowConstraints,
      std::make_shared<squiggles::TankModel>(
          chassisScales.wheelTrack.convert(1_m), slowConstraints),
      0.01);

  auto shoot = [&](std::size_t numTimes = 1) {
    for (size_t i = 0; i < numTimes; i++) {
      flywheel->waitUntilSettled();
      cylinder.set_value(1);
      pros::delay(300);
      cylinder.set_value(0);
      pros::delay(50);
    }
  };
  constexpr const auto targetPoint = okapi::Point{17.39_in, 122.24_in};
  constexpr const auto closeTargetPoint = okapi::Point({122.24_in, 17.39_in});
  constexpr const auto rollerCrossHair = okapi::Point({110.59_in, 110.59_in});
  constexpr const auto shotOffset = 2_deg;
  auto flipRoller = [=](bool reverse = false, okapi::QAngle amount = 120_deg) {
    int reverser = reverse ? -1 : 1;
    intake->flipRaw(amount * reverser, 400);
  };

  auto shootToPoint = [=](okapi::Point point, std::size_t numshots) {
    flywheel->setTarget(
        distanceCalcRPM(distanceToPoint(point, odometry->getPoint())));
    chassis->turnToPoint(point, shotOffset);
    shoot(numshots);
  };

  switch (display.getAuton()) {
    case 1:  // Full Cross Field from left
      flywheel->setTarget(
          distanceCalcRPM(distanceToPoint(targetPoint, odometry->getPoint())));
      intake->setLoopSkip(true);
      odometry->setState({32_in, 12_in, -90_deg});

      // Roller
      chassis->moveRaw(0.25, 150);
      flipRoller();
      // Shot #1
      chassis->driveToPoint({32_in, 14_in}, true, 0.5_in);
      shootToPoint(targetPoint, 2);
      intake->setLoopSkip(false);  // Off of roller, can re-enable
      intake->setManualMode(true);
      intake->setEnabledMode(true);

      // // Shot #2
      // chassis->driveToPoint({3_tile, 2_tile});
      // shootToPoint(targetPoint, 3);

      // // Shot #3
      // chassis->driveToPoint({4.5_tile, 3.5_tile});
      // shootToPoint(targetPoint, 3);

      // To roller
      chassis->driveToPoint({5.25_tile, 4.6_tile});
      intake->setLoopSkip(true);  // Entering roller area
      chassis->turnRaw(1, 1000);
      chassis->turnToAngle(7.5_deg);
      chassis->waitUntilSettled();
      intake->setManualMode(false);
      intake->setEnabledMode(false);
      intake->setLoopSkip(true);
      chassis->moveRaw(0.5, 600);
      flipRoller();
      // Match setup
      // chassis->driveToPoint({5_tile, 4_tile}, true);
      chassis->moveRaw(-0.4, 225);
      intake->setLoopSkip(false);

      flywheel->setTarget(
          distanceCalcRPM(distanceToPoint(targetPoint, odometry->getPoint())));
      chassis->turnToPoint(targetPoint);
      flywheel->setTarget(
          distanceCalcRPM(distanceToPoint(targetPoint, odometry->getPoint())));
      shoot(3);
      break;
    case 2:  // Right side mid
      odometry->setState({5_tile + 14.5_in / 2, 3.5_tile, 90_deg});
      intake->setLoopSkip(true);
      // Shot #1
      shootToPoint(targetPoint, 2);
      // Roller
      chassis->driveToPoint({5_tile + 14.5_in / 2, 4.6_tile});
      chassis->turnToAngle(0_deg);
      chassis->waitUntilSettled();
      chassis->moveRaw(0.5, 500);
      flipRoller();
      // Back up
      chassis->driveDistance(-4_in);
      chassis->waitUntilSettled();
      intake->setLoopSkip(false);
      intake->setEnabledMode(true);
      intake->setManualMode(false);
      // Shot #2
      chassis->driveToPoint({3.5_tile, 2.5_tile}, 0.7);
      shootToPoint(targetPoint, 3);
      // Shot #3
      chassis->driveToPoint({2.5_tile, 1.5_tile}, 0.4);
      shootToPoint(targetPoint, 3);
      break;
    case 3:  // Right roller only
      odometry->setState({5_tile + 14.5_in / 2, 3.5_tile, 90_deg});
      shootToPoint(targetPoint, 2);
      intake->setLoopSkip(true);
      // Roller
      chassis->driveToPoint({5_tile + 14.5_in / 2, 4.6_tile});
      chassis->turnToAngle(0_deg);
      chassis->waitUntilSettled();
      chassis->moveRaw(0.5, 300);
      flipRoller();
      // Match setup
      chassis->driveDistance(-4_in);
      chassis->waitUntilSettled();
      chassis->turnToAngle(180_deg);
      chassis->waitUntilSettled();

      break;
    case 4:  // Left roller only
      flywheel->setTarget(
          distanceCalcRPM(distanceToPoint(targetPoint, odometry->getPoint())));
      intake->setLoopSkip(true);
      odometry->setState({32_in, 12_in, -90_deg});

      // Roller
      chassis->moveRaw(0.25, 150);
      flipRoller();
      // Shot #1
      chassis->driveToPoint({32_in, 14_in}, true, 0.5_in);
      shootToPoint(targetPoint, 2);
      intake->setLoopSkip(false);  // Off of roller, can re-enable
      break;
    case 5:  // Left mid
      flywheel->setTarget(
          distanceCalcRPM(distanceToPoint(targetPoint, odometry->getPoint())));
      intake->setLoopSkip(true);
      odometry->setState({32_in, 12_in, -90_deg});

      // Roller
      chassis->moveRaw(0.25, 150);
      flipRoller();
      // Shot #1
      chassis->driveToPoint({32_in, 14_in}, true, 0.5_in);
      shootToPoint(targetPoint, 2);
      intake->setLoopSkip(false);  // Off of roller, can re-enable

      // // Shot #2
      chassis->driveToPoint({3_tile, 2_tile}, 0.4);
      shootToPoint(targetPoint, 3);

      // // Shot #3
      chassis->driveToPoint({4.5_tile, 3.5_tile}, 0.7);
      shootToPoint(targetPoint, 3);

    case 6:  // Prog skills

      flywheel->setTarget(
          distanceCalcRPM(distanceToPoint(targetPoint, odometry->getPoint())));
      intake->setLoopSkip(true);
      odometry->setState({32_in, 12_in, -90_deg});

      // Roller
      chassis->moveRaw(0.25, 150);
      flipRoller(false, 220_deg);
      intake->setLoopSkip(false);
      // Shot #1
      chassis->driveToPoint({32_in, 14_in}, true, 0.5_in);
      shootToPoint(closeTargetPoint, 2);

      // Shot #2
      chassis->driveToPoint({2.7_tile, 1.7_tile});
      shootToPoint(closeTargetPoint, 3);

      // Shot #3
      chassis->driveToPoint({4.7_tile, 3.7_tile});
      shootToPoint(closeTargetPoint, 3);

      // Rollers
      chassis->driveToPoint(rollerCrossHair);
      chassis->turnToAngle(0_deg);
      intake->setLoopSkip(true);
      chassis->waitUntilSettled();
      chassis->moveRaw(0.5, 900);
      flipRoller(false, 220_deg);
      chassis->driveDistance(-6_in);
      chassis->waitUntilSettled();
      chassis->driveToPoint(rollerCrossHair, true);

      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();
      chassis->moveRaw(0.5, 900);
      flipRoller(true, 220_deg);
      chassis->driveDistance(-6_in);
      chassis->waitUntilSettled();
      intake->setLoopSkip(false);

      chassis->driveToPoint({4.25_tile, 3.25_tile});
      shootToPoint(targetPoint, 3);

      chassis->driveToPoint({2.25_tile, 1.25_tile});
      shootToPoint(targetPoint, 3);

      chassis->driveToPoint({1_tile, 1.5_tile});
      shootToPoint(closeTargetPoint, 3);

      chassis->turnToAngle(40_deg);
      chassis->waitUntilSettled();
      pros::c::adi_digital_write(2, true);
      chassis->turnToAngle(50_deg);
      chassis->waitUntilSettled();
      pros::c::adi_digital_write(4, true);

      break;
    case 7:  // Testing auton
      flywheel->setTarget(0);
      odometry->setState({0_in, 0_in, 90_deg});
      // chassis->driveToPointByPath({0_tile, 2_tile, 90_deg}, fastGen);
      // chassis->waitUntilSettled();
      // chassis->driveToPointByPath({2_tile, 2_tile, 0_deg}, fastGen, true);
      // chassis->waitUntilSettled();
      // chassis->driveToPointByPath({2_tile, 0_tile, -90_deg}, fastGen, true);
      // chassis->waitUntilSettled();
      // chassis->driveToPointByPath({0_tile, 0_tile, 180_deg}, fastGen, true);
      // chassis->waitUntilSettled();
      // chassis->drivePathFromHere({{0_tile, 1_tile, 90_deg},
      //                             {1_tile, 2_tile, 0_deg},
      //                             {2_tile, 1_tile, 270_deg},
      //                             {1_tile, 0_tile, 180_deg},
      //                             {0_tile, 0_tile, 180_deg}},
      //                            fastGen);
      // chassis->waitUntilSettled();

      chassis->driveToPoint({0_tile, 2_tile}, 0.4);
      chassis->driveToPoint({2_tile, 2_tile}, 0.4);
      chassis->driveToPoint({2_tile, 0_tile}, 0.4);
      chassis->driveToPoint({0_tile, 0_tile}, 0.4);

    default:  // Disabled or unkown index
      break;
  }
  flywheel->setTarget(0);
  intake->setManualMode(true);
  intake->setEnabledMode(false);
  chassis->cancelMovement();
}

void opcontrol() {
// #define tuning
#define quiet

  using std::literals::string_literals::operator""s;
  controller.clear();
  pros::delay(50);

  pros::Task trigger([=]() {
    pros::ADIDigitalOut cylinder(1, false);
    okapi::ControllerButton trigger(okapi::ControllerDigital::R2);
    int shotCount = 0;
    while (pros::Task::notify_take(true, 10) == 0U) {
      if (trigger.isPressed() && flywheel->isSettled()) {
        controller.setText(1, 0, std::to_string(++shotCount));
        cylinder.set_value(1);
        pros::delay(300);
        cylinder.set_value(0);
      }
    }
  });

  pros::c::task_notify_when_deleting(
      pros::c::task_get_current(), (pros::task_t)trigger, 1,
      pros::notify_action_e_t::E_NOTIFY_ACTION_OWRITE);

  std::atomic_bool canExpand = false;
  pros::Task timer([=, &canExpand]() {
    pros::delay(95000);
    canExpand = true;
    controller.rumble("-"s);
  });

  pros::c::task_notify_when_deleting(
      pros::c::task_get_current(), (pros::task_t)timer, 1,
      pros::notify_action_e_t::E_NOTIFY_ACTION_OWRITE);

  controller.rumble("-"s);  // Match start rumble

  // flywheel->setTarget(distanceCalcRPM(110_in));
  // flywheel->setTarget(0);
  intake->setLoopSkip(false);
  intake->setManualMode(false);
  intake->setEnabledMode(true);
  okapi::ControllerButton up(okapi::ControllerDigital::X);
  okapi::ControllerButton down(okapi::ControllerDigital::B);

  pros::ADIDigitalOut leftCyl{4, false};
  pros::ADIDigitalOut rightCyl{2, false};

#ifndef tuning
#ifndef quiet
  flywheel->setTarget(distanceCalcRPM(110_in));
#endif
#endif

#ifdef tuning
  constexpr auto size = 3;
  auto getter = [=]() -> std::array<double, size> {
    auto [P, I, D, B] = distancePID->getGains();
    return std::array{P, I, D};
  };
  auto setter = [=](std::array<double, 3> gains) -> void {
    distancePID->setGains({gains[0], gains[1], gains[2]});
    std::cout << "P" << gains[0] << "\nI" << gains[1] << "\nD" << gains[2]
              << "\n";
  };

  auto pidTuner = PIDConstantsTuner<3, getter, setter>();
  int direction = 1;

#endif

  while (true) {
#ifndef tuning
    if (controller[okapi::ControllerDigital::L1].changedToPressed() &&
        canExpand) {
      rightCyl.set_value(1);
    }
    if (controller[okapi::ControllerDigital::L2].changedToPressed() &&
        canExpand) {
      leftCyl.set_value(1);
    }

    if (up.changedToPressed()) {
      flywheel->setTarget(distanceCalcRPM(110_in));
    }
    if (down.changedToPressed()) {
      flywheel->setTarget(distanceCalcRPM(30_in));
    }

    if (controller[okapi::ControllerDigital::down].changedToPressed()) {
      intake->toggleManualMode();
    }
    if (controller[okapi::ControllerDigital::right].changedToPressed()) {
      intake->toggleEnabledMode();
    }

    display.setPosition(odometry->getState());

    model->driveVector(controller.getAnalog(okapi::ControllerAnalog::leftY),
                       controller.getAnalog(okapi::ControllerAnalog::rightX));

#endif
#ifdef tuning
    if (controller[okapi::ControllerDigital::X].changedToPressed())
      pidTuner.inc();
    if (controller[okapi::ControllerDigital::B].changedToPressed())
      pidTuner.dec();
    if (controller[okapi::ControllerDigital::up].changedToPressed())
      pidTuner.decPresicion();
    if (controller[okapi::ControllerDigital::down].changedToPressed())
      pidTuner.incPrecision();
    if (controller[okapi::ControllerDigital::right].changedToPressed())
      pidTuner.nextGain();
    if (controller[okapi::ControllerDigital::L1].changedToPressed()) {
      chassis->driveDistance(24_in * direction);
      direction *= -1;
      while (!chassis->isSettled() &&
             !controller[okapi::ControllerDigital::L2].changedToPressed()) {
        pros::delay(10);
      }
      chassis->cancelMovement();
      controller.rumble(".");
    };

#endif

    pros::delay(10);
  }
}