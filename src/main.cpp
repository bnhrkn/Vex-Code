#include "main.h"
#include "project/CustomChassisController.hpp"
#include "project/DiscStack.hpp"
#include "project/pidTuner.hpp"
#include "project/ramsete.hpp"
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
std::shared_ptr<Indexer> indexer;
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

  indexer = std::make_shared<Indexer>(pros::Distance(13),
                                      pros::ADIDigitalOut(1), 150);

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

  pros::Task intake{[=] {
    pros::Motor intake(-4);
    pros::Optical colorSensor(12);
    colorSensor.set_led_pwm(255);
    intake.set_gearing(pros::E_MOTOR_GEAR_GREEN);
    constexpr auto proxThreshold = 20;

    okapi::ControllerButton forward(okapi::ControllerDigital::right);
    okapi::ControllerButton reverse(okapi::ControllerDigital::left);
    okapi::ControllerButton autoPilotBtn(okapi::ControllerDigital::down);

    auto inRange = [](auto num,
                      std::pair<decltype(num), decltype(num)> range) -> bool {
      auto minMax = std::minmax(range.first, range.second);
      return (num > std::get<0>(minMax)) && (num < std::get<1>(minMax));
    };

    std::pair<int, int> blueRange = {220, 240};
    decltype(blueRange) redRange = {0, 25};

    auto hueRange = display.isBlueTeam() ? blueRange : redRange;

    bool intakeEnabled = true;
    auto autoPilotEnabled = true;

    while (!pros::Task::notify_take(true, 20)) {
      // Toggle Controls
      if (autoPilotBtn.changedToPressed()) {
        autoPilotEnabled = !autoPilotEnabled;
      }
      if (forward.changedToPressed()) {
        intakeEnabled = !intakeEnabled;
      }

      if (reverse.isPressed()) { // Reverse at any time
        intake.move_velocity(-200);
        continue;
      }
      if (!autoPilotEnabled) { // Manual mode
                               // Only spin if enabled
        intake.move_velocity(static_cast<int>(intakeEnabled) * 200);
        continue;
      }
      // Autopilot code

      // Intake should run and no roller detected
      auto hue = colorSensor.get_hue();
      auto prox = colorSensor.get_proximity();

      if ((indexer->getCount() < 3 ||
           intake.get_power() > 4.5) && // Intake should run
          ((prox > proxThreshold) &&    // Far away
           !(inRange(hue, blueRange) ||
             inRange(hue, redRange)))) { // Not red or blue

        if (intake.get_efficiency() < 10) {
          intake.move_velocity(-200);
          pros::delay(200);
        } else {
          intake.move_velocity(200);
        }
      } else if (prox <= proxThreshold &&  // Close
                 inRange(hue, hueRange)) { // Right color
        intake.move_velocity(200);
      } else {
        intake.move_velocity(0);
      }
    }
  }};

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

  auto shoot = [&]() {
    flywheel->waitUntilSettled();
    cylinder.set_value(true);
    pros::delay(300);
    cylinder.set_value(false);
    pros::delay(50);
  };

  flywheel->setTarget(505);

  // X is pos horizontal, Y is pos vertical, theta is pos counterclockwise and 0
  // is on x axis
  // Begin at (36_in, 12_in, -90_deg)
  // odometry->setState(convertState({36_in, 12_in, -90_deg}));
  odometry->setState(convertState({0_in, 0_in, -90_deg}));

  // chassis->runPath(generatePathTo({{36_in, 10_in, -90_deg}}));
  // chassis->runPath(fastGen.generate({{0,0,0},{0.6096,0,0}}));
  // chassis->runPath(generatePath({{0_in,0_in,0_deg}, {24_in,0_in,45_deg},
  // {24_in, 24_in, 90_deg}}, fastGen));
  chassis->driveDistance(2_in);
  chassis->waitUntilSettled();
  // intake.move_relative(180 / ((36.0 / 84.0) * (12.0 / 24.0)), 200);
  chassis->driveDistance(-2_in);
  chassis->waitUntilSettled();
  chassis->turnToAngle(100.5_deg);
  chassis->waitUntilSettled();
  shoot();
  shoot();
  chassis->turnToAngle(45_deg);
  chassis->waitUntilSettled();
  chassis->driveDistance(34_in);
  chassis->waitUntilSettled();
  chassis->turnToAngle(115.5_deg);
  chassis->waitUntilSettled();
  shoot();
  shoot();
  shoot();
  chassis->turnToAngle(45_deg);
  chassis->waitUntilSettled();
  // chassis->driveDistance(102_in);
  // chassis->waitUntilSettled();
  // chassis->turnToAngle(0_deg);
  // chassis->waitUntilSettled();

  // //chassis->runPath(generatePathTo({{36_in, 12_in, -90_deg}}));
  // chassis->runPath(generatePath({{2_in,0_in,0_deg},{0_in,0_in,0_deg}}));

  // chassis.turnToAngle(90_deg);
  // chassis.waitUntilSettled();
  // while (true) {
  //   display.setPosition(getConvertedState(odometry));
  //   pros::delay(10);
  // }

  // pros::delay(750);
  // chassis->moveDistance(-4_in);
  // chassis->turnAngle(185_deg);

  // flywheel->waitUntilSettled();
  // cylinder.set_value(true);
  // pros::delay(300);
  // cylinder.set_value(false);

  // flywheel->waitUntilSettled();
  // cylinder.set_value(true);
  // pros::delay(300);
  // cylinder.set_value(false);

  // flywheel->setTarget(0);
}

void opcontrol() {
  using std::literals::string_literals::operator""s;

  pros::Task matchTimer{[&] {
    okapi::ControllerButton expand(okapi::ControllerDigital::Y);
    pros::ADIDigitalOut cylinder(3, false);
    controller.rumble("-"s); // Match start rumble
    pros::delay(95000);      // Delay until 1:35
    controller.rumble("-"s); // Rumble on endgame start at 1:35
    while (true) {
      if (expand.changedToPressed()) {
        cylinder.set_value(true);
      }
    }
  }};

  pros::Task trigger{[&] { // Task for the disc shooter trigger
    pros::ADIDigitalOut cylinder(1, false);
    okapi::ControllerButton trigger(okapi::ControllerDigital::R2);
    int count = 0;
    while (true) {
      if (trigger.isPressed() && flywheel->isSettled()) {
        controller.setText(1, 0, std::to_string(++count));

        cylinder.set_value(true);
        pros::delay(300);
        cylinder.set_value(false);
      }
      pros::delay(10);
    }
  }};

  pros::Task tilter([=] {
    flywheel->setTarget(505); // 0.885
     // flywheel->setTarget(600 * 0.80); // 0.87 far shot normal 42 deg / 30
     okapi::ControllerButton up(okapi::ControllerDigital::X);
     okapi::ControllerButton down(okapi::ControllerDigital::B);
     //pros::ADIDigitalOut cyl(2, true);
     pros::ADIDigitalOut cyl(2, false);
     bool high = true;
     while (true) {
       if (up.changedToPressed() && !high) {
         high = true;
         flywheel->setTarget(505);
       } else if (down.changedToPressed() && high) {
         high = false;
         flywheel->setTarget(400);
       }
       pros::delay(20);
     }
  });

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
    // odometry->step();
    display.setPosition(getConvertedState(odometry));

    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  -controller.getAnalog(okapi::ControllerAnalog::rightX));

    pros::delay(10);
  }
}