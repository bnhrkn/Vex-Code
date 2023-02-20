#include "main.h"
#include "project/CustomChassisController.hpp"
#include "project/algorithms.hpp"
#include "project/intake.hpp"
#include "project/pidTuner.hpp"
#include "project/ui.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>

using namespace okapi::literals;

okapi::Controller controller;

const auto ratio = okapi::AbstractMotor::GearsetRatioPair{
    okapi::AbstractMotor::gearset::blue, 36.0 / 60.0}; // motor rpm to wheel rpm
const auto constraints = squiggles::Constraints(2, 2, 9);
const auto slowConstraints = squiggles::Constraints(0.5, 2, 9);
const auto odomScales =
    okapi::ChassisScales({2.75_in, 7.35_in, 4.50_in, 2.75_in}, 360);
const auto chassisScales =
    okapi::ChassisScales({3.25_in, 12.75_in}, okapi::imev5BlueTPR);

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

auto angleGains = okapi::IterativePosPIDController::Gains{0.0052, 0.000, 0.0};
auto distanceGains = okapi::IterativePosPIDController::Gains{0.6, 0.0, 0.0};

std::shared_ptr<okapi::ThreeEncoderSkidSteerModel> model;
std::shared_ptr<okapi::AsyncVelIntegratedController> flywheel;
std::shared_ptr<okapi::ThreeEncoderOdometry> odometry;
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
      okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
      "/ser/sout",                  // Output to the PROS terminal
      okapi::Logger::LogLevel::warn // Show errors and warnings
      ));

  std::pair<int, int> blueRange = {220, 240};
  decltype(blueRange) redRange = {0, 25};

  auto hueRange = display.isBlueTeam() ? blueRange : redRange;
  auto wrongHueRange = display.isBlueTeam() ? redRange : blueRange;

  intake = std::make_shared<Intake>(
      pros::Motor(-4), pros::Optical(16), pros::Distance(13), hueRange,
      wrongHueRange, okapi::ControllerButton(okapi::ControllerDigital::left));

  flywheel = std::make_shared<okapi::AsyncVelIntegratedController>(
      std::make_shared<okapi::Motor>(8),
      okapi::AbstractMotor::GearsetRatioPair(
          okapi::AbstractMotor::gearset::blue, 1),
      3600, okapi::TimeUtilFactory::withSettledUtilParams(1, 50, 0_ms));

  std::ranges::for_each(std::array{14, 7, 6}, pros::c::rotation_reset_position);

  model = std::make_shared<okapi::ThreeEncoderSkidSteerModel>(
      std::make_shared<okapi::MotorGroup>(okapi::MotorGroup{
          makeMotor(1, false), makeMotor(2, true), makeMotor(3, false)}),
      std::make_shared<okapi::MotorGroup>(okapi::MotorGroup{
          makeMotor(11, true), makeMotor(12, false), makeMotor(17, true)}),

      std::make_shared<okapi::RotationSensor>(14, true),       // left
      std::make_shared<okapi::RotationSensor>(7, false),       // right
      std::make_shared<okapi::RotationSensor>(6), 600, 12000); // middle

  model->resetSensors();

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  odometry = std::make_shared<okapi::ThreeEncoderOdometry>(
      okapi::TimeUtilFactory::createDefault(), model, odomScales);

  odometry->setState(convertState({0_in, 0_in, -90_deg}));

  turnPID = std::make_shared<okapi::IterativePosPIDController>(
      angleGains,
      okapi::ConfigurableTimeUtilFactory(0.75, 0.5, 100_ms).create());

  distancePID = std::make_shared<okapi::IterativePosPIDController>(
      distanceGains,
      okapi::ConfigurableTimeUtilFactory(0.01, 0.02, 100_ms).create());

  chassis = std::make_shared<CustomChassisController>(
      model, turnPID, distancePID, odometry, chassisScales, ratio);

  pros::delay(
      500); // There is a race condition somewhere, sensors fail to reset.
}

void disabled() { model->stop(); }

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

  auto generatePath = [](std::vector<okapi::OdomState> states,
                         squiggles::SplineGenerator &generator) {
    std::vector<squiggles::Pose> poses(states.size());
    std::ranges::transform(states, poses.begin(), stateToPose);
    return generator.generate(poses);
  };
  auto generatePathTo = [&](std::vector<okapi::OdomState> states,
                            squiggles::SplineGenerator &generator) {
    states.insert(states.cbegin(), getConvertedState(odometry));
    return generatePath(states, generator);
  };

  auto shoot = [&](int numTimes = 1) {
    for (int i = 0; i < numTimes; i++) {
      flywheel->waitUntilSettled();
      cylinder.set_value(true);
      pros::delay(300);
      cylinder.set_value(false);
      pros::delay(50);
    }
  };
  constexpr auto targetPoint = okapi::Point{0.75_tile, 5.25_tile};

  flywheel->setTarget(505);

  switch (display.getAuton()) {
  case 1: // Full Cross Field from left
    odometry->setState(convertState({32_in, 12_in, -90_deg}));
    chassis->driveDistance(2_in);
    // Turn the roller
    intake->flipRaw(90_deg);
    intake->waitUntilSettled();
    chassis->driveToPoint({32_in, 12_in}, true);
    chassis->turnToPoint(targetPoint);
    shoot(2);
    chassis->driveToPoint({2.5_tile, 1.5_tile});
    chassis->turnToPoint(targetPoint);
    shoot(3);
    chassis->driveToPoint({4.5_tile, 3.5_tile});
    chassis->turnToPoint(targetPoint);
    shoot(3);
    chassis->driveToPoint({5.5_tile, 4.7_tile});
    chassis->turnToAngle(0_deg);
    chassis->driveDistance(4_in);
    // Turn the roller
    intake->flipRaw(90_deg);
    intake->waitUntilSettled();
    chassis->driveToPoint({5.5_tile, 4.7_tile}, true);
    chassis->turnToAngle(180_deg);
    break;
  case 2: // Right side full
    break;
  case 3: // Right roller only
    break;
  case 4: // Left roller
    odometry->setState(convertState({32_in, 12_in, -90_deg}));
    chassis->driveDistance(2_in);
    // Turn the roller
    intake->flipRaw(90_deg);
    intake->waitUntilSettled();
    chassis->driveToPoint({32_in, 12_in}, true);
    chassis->turnToPoint(targetPoint);
    shoot(2);
    break;
  case 5: // Left full
    odometry->setState(convertState({32_in, 12_in, -90_deg}));
    chassis->driveDistance(2_in);
    // Turn the roller
    intake->flipRaw(90_deg);
    intake->waitUntilSettled();
    chassis->driveToPoint({32_in, 12_in}, true);
    chassis->turnToPoint(targetPoint);
    shoot(2);
    chassis->driveToPoint({2.5_tile, 1.5_tile});
    chassis->turnToPoint(targetPoint);
    shoot(3);
    chassis->driveToPoint({4.5_tile, 3.5_tile});
    chassis->turnToPoint(targetPoint);
    shoot(3);
    break;
  default: // Disabled or unkown index
    break;
  }
}

void opcontrol() {
  using std::literals::string_literals::operator""s;

  pros::Task trigger([=]() {
    pros::ADIDigitalOut cylinder(1, false);
    okapi::ControllerButton trigger(okapi::ControllerDigital::R2);
    int shotCount = 0;
    while (!pros::Task::notify_take(true, 10)) {
      if (trigger.changedToPressed() && flywheel->isSettled()) {
        controller.setText(1, 0, std::to_string(++shotCount));
        cylinder.set_value(true);
        pros::delay(300);
        cylinder.set_value(false);
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

  controller.rumble("-"s); // Match start rumble
  okapi::ControllerButton expandBtn(okapi::ControllerDigital::Y);
  pros::ADIDigitalOut expandCylinder(3, false);

  flywheel->setTarget(505);
  okapi::ControllerButton up(okapi::ControllerDigital::X);
  okapi::ControllerButton down(okapi::ControllerDigital::B);
  bool high = true;

  okapi::ControllerButton pidSelector(okapi::ControllerDigital::R1);
  okapi::ControllerButton incPrecision(okapi::ControllerDigital::A);
  okapi::ControllerButton decPresicion(okapi::ControllerDigital::Y);
  okapi::ControllerButton inc(okapi::ControllerDigital::X);
  okapi::ControllerButton dec(okapi::ControllerDigital::B);

  okapi::ControllerButton move(okapi::ControllerDigital::L1);

  constexpr size_t size = 3;

  auto gainGetter = [=]() {
    auto gains = distancePID->getGains();
    return std::array<double, size>{gains.kP, gains.kI, gains.kD};
  };

  auto gainSetter = [=](std::array<double, size> arr) {
    std::cout << "P: " << arr[0] << "\nI: " << arr[1] << "\nD: " << arr[2]
              << std::endl;

    distancePID->setGains({arr[0], arr[1], arr[2]});
  };
  auto bruh = PIDConstantsTuner<size, gainGetter, gainSetter>();

  controller.clear();

  while (true) {

    if (expandBtn.changedToPressed() && canExpand) {
      expandCylinder.set_value(true);
    }

    if (up.changedToPressed() && !high) {
      high = true;
      flywheel->setTarget(505);
    } else if (down.changedToPressed() && high) {
      high = false;
      flywheel->setTarget(400);
    }

    if (dec.changedToPressed())
      bruh.dec();
    if (inc.changedToPressed())
      bruh.inc();
    if (decPresicion.changedToPressed())
      bruh.decPresicion();
    if (incPrecision.changedToPressed())
      bruh.incPrecision();
    if (pidSelector.changedToPressed()) {
      bruh.nextGain();
    }

    if (controller[okapi::ControllerDigital::down].changedToPressed())
      intake->toggleManualMode();
    if (controller[okapi::ControllerDigital::right].changedToPressed())
      intake->toggleEnabledMode();
    // odometry->step();
    display.setPosition(getConvertedState(odometry));

    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  -controller.getAnalog(okapi::ControllerAnalog::rightX));

    pros::delay(10);
  }
}