#include "main.h"
//#include "project/CustomChassisController.hpp"
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

constinit const auto gearset = okapi::AbstractMotor::gearset::blue;
const auto ratio = okapi::AbstractMotor::GearsetRatioPair{gearset, 36.0 / 60.0};
constinit const auto encoderUnit = okapi::AbstractMotor::encoderUnits::counts;
constinit const auto maxRPM = okapi::toUnderlyingType(gearset);

using gains = okapi::IterativePosPIDController::Gains;
auto linearGains = gains{0.0001, 0.001, 0.001};
auto courseGains = gains{0.0001, 0.001, 0.001};
auto angleGains = gains{0.0001, 0.001, 0.001};

std::shared_ptr<okapi::ThreeEncoderSkidSteerModel> model;

const auto odomScales =
    okapi::ChassisScales({2.75_in, 7.35_in, 4.50_in, 2.75_in}, 360);

const auto chassisScales =
    okapi::ChassisScales({3.25_in, 12.75_in}, okapi::imev5BlueTPR);

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

std::shared_ptr<okapi::AsyncVelIntegratedController> flywheel;

void on_center_button() {}

void initialize() {
  auto makeMotor = [](int iport, bool ireversed) {
    return okapi::Motor(iport, ireversed, gearset, encoderUnit);
  };
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
      "/ser/sout",                  // Output to the PROS terminal
      okapi::Logger::LogLevel::warn // Show errors and warnings
      ));

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

      std::make_shared<okapi::RotationSensor>(14, true),          // left
      std::make_shared<okapi::RotationSensor>(7, false),          // right
      std::make_shared<okapi::RotationSensor>(6), maxRPM, 12000); // middle

  model->resetSensors();

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  pros::delay(
      500); // There is a race condition somewhere, sensors fail to reset.
}

void disabled() { model->stop(); }

void competition_initialize() {}

void autonomous() {
  auto chassis =
      okapi::ChassisControllerBuilder()
          .withMotors({1, -2, 3}, {-11, 12, -17})
          .withDimensions(ratio, odomScales)
          .withSensors(okapi::RotationSensor(
                           7, false), // swapped the left and right sensors?
                       okapi::RotationSensor(14, true))
          .withOdometry(odomScales)
          .withGains({0.0006, 0.0000, 0.000000}, {0.0019, 0.00061, 0.000001},
                     {0.0009, 0.00031, 0.000001})
          .withChassisControllerTimeUtilFactory(
              okapi::ConfigurableTimeUtilFactory(0.5, 1.0, 200_ms))
          .build();
  auto model = chassis->getModel();

  pros::Motor intake (-4);
  pros::ADIDigitalOut cylinder(1, false);
  pros::ADIDigitalOut tilter(2, true);

  flywheel->setTarget(600 * 0.87);

  chassis->moveDistance(2_in);
  intake.move_relative(180 / ((36.0/84.0) * (12.0/24.0)), 200);
  pros::delay(750);
  chassis->moveDistance(-4_in);
  chassis->turnAngle(185_deg);

  flywheel->waitUntilSettled();
  cylinder.set_value(true);
  pros::delay(300);
  cylinder.set_value(false);

  flywheel->waitUntilSettled();
  cylinder.set_value(true);
  pros::delay(300);
  cylinder.set_value(false);

  flywheel->setTarget(0);
  // chassis->moveDistance(6_ft);
  // chassis->turnAngle(180_deg);
  // chassis->moveDistance(6_ft);
}

void opcontrol() {
  auto chassis =
      okapi::ChassisControllerBuilder()
          .withMotors({1, -2, 3}, {-11, 12, -17})
          .withDimensions(ratio, odomScales)
          .withSensors(okapi::RotationSensor(
                           7, false), // swapped the left and right sensors?
                       okapi::RotationSensor(14, true))
          .withOdometry(odomScales)
          .withGains({0.0006, 0.0000, 0.000000}, {0.0019, 0.00061, 0.000001},
                     {0.0009, 0.00031, 0.000001})
          .withChassisControllerTimeUtilFactory(
              okapi::ConfigurableTimeUtilFactory(0.5, 1.0, 200_ms))
          .build();
  auto chassisPID =
      std::dynamic_pointer_cast<okapi::ChassisControllerPID>(chassis);

  using std::literals::string_literals::operator""s;

  pros::Task matchTimer{[&] {
    okapi::ControllerButton expand(okapi::ControllerDigital::right);
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

  pros::Task intake{[&] {
    pros::Motor intake(-4);
    pros::Optical colorSensor(12);
    colorSensor.set_led_pwm(255);

    okapi::ControllerButton intakeBtn(okapi::ControllerDigital::L2);
    okapi::ControllerButton reverse(okapi::ControllerDigital::left);
    intake.set_gearing(pros::E_MOTOR_GEAR_GREEN);
    auto [hueMin, hueMax] = [&]() -> std::pair<int, int> {
      if (display.isBlueTeam()) {
        return {220, 240}; // Blue Path
      } else {
        return {0, 25}; // Red Path
      }
    }();

    enum class intakeMode { fast, off };
    auto mode = intakeMode::fast;
    while (true) {
      if (intakeBtn.changedToPressed()) {
        if (mode == intakeMode::fast) {
          mode = intakeMode::off;
        } else {
          mode = intakeMode::fast;
        }
      }

      if (colorSensor.get_hue() >= hueMin && colorSensor.get_hue() <= hueMax) {
        intake.move_velocity(0);
      } else {
        if (reverse.isPressed()) {
          intake.move_velocity(-200);
        } else if (mode == intakeMode::off) {
          intake.move_velocity(0);
        } else {
          intake.move_velocity(200);
        }
      }
      pros::delay(20);
    }
  }};

  pros::Task tilter([=] {
    flywheel->setTarget(505); //0.885
    // flywheel->setTarget(600 * 0.80); // 0.87 far shot normal 42 deg / 30 deg
    okapi::ControllerButton up(okapi::ControllerDigital::X);
    okapi::ControllerButton down(okapi::ControllerDigital::B);
    //pros::ADIDigitalOut cyl(2, true);
    pros::ADIDigitalOut cyl(2, false);
    bool high = true;
    while (true) {
      if (up.changedToPressed() && !high) {
        high = true;
        flywheel->setTarget(505);
        //cyl.set_value(true);
      } else if (down.changedToPressed() && high) {
        high = false;
        flywheel->setTarget(400);
        //cyl.set_value(false);
      }
      pros::delay(20);
    }
  });

  int gainType = 0;
  int precisionLevel = 0;
  double change = 0.001;
  okapi::ControllerButton pidSelector(okapi::ControllerDigital::Y);
  okapi::ControllerButton precisionSelector(okapi::ControllerDigital::A);
  okapi::ControllerButton inc(okapi::ControllerDigital::up);
  okapi::ControllerButton dec(okapi::ControllerDigital::down);

  while (true) {
    // if (pidSelector.changedToPressed()) {
    //   ++gainType %= 3;
    // }
    // if (precisionSelector.changedToPressed()) {
    //   ++precisionLevel %= 3;
    //   switch (precisionLevel) {
    //   case 0:
    //     change = 0.001;
    //     break;
    //   case 1:
    //     change = 0.0001;
    //     break;
    //   case 2:
    //     change = 0.00001;
    //     break;
    //   }
    // }
    // if (inc.changedToPressed()) {
    //   auto gains = chassisPID->getGains();
    //   auto gain = std::get<1>(gains);
    //   switch (gainType) {
    //   case 1: // integral
    //     gain.kI += change;
    //     controller.setText(1, 0, "kI: "s + std::to_string(gain.kI));
    //     break;
    //   case 2: // derivative
    //     gain.kD += change;
    //     controller.setText(1, 0, "kD: "s + std::to_string(gain.kD));
    //     break;
    //   default: // proportional
    //     gain.kP += change;
    //     controller.setText(1, 0, "kP: "s + std::to_string(gain.kP));
    //   }
    //   chassisPID->setGains(std::get<0>(gains), gain, std::get<2>(gains));
    // }

    // if (dec.changedToPressed()) {
    //   auto gains = chassisPID->getGains();
    //   auto gain = std::get<1>(gains);
    //   switch (gainType) {
    //   case 1: // integral
    //     gain.kI -= change;
    //     controller.setText(1, 0, "kI: "s + std::to_string(gain.kI));
    //     break;
    //   case 2: // derivative
    //     gain.kD -= change;
    //     controller.setText(1, 0, "kD: "s + std::to_string(gain.kD));
    //     break;
    //   default: // proportional
    //     gain.kP -= change;
    //     controller.setText(1, 0, "kP: "s + std::to_string(gain.kP));
    //   }
    //   chassisPID->setGains(std::get<0>(gains), gain, std::get<2>(gains));
    // }

    if(inc.changedToPressed()){
      flywheel->setTarget(flywheel->getTarget() + 1);
      controller.setText(1,0, "Fly Target: "s + std::to_string(flywheel->getTarget()));
    }
    if(dec.changedToPressed()){
      flywheel->setTarget(flywheel->getTarget() - 1);
      controller.setText(1,0, "Fly Target: "s + std::to_string(flywheel->getTarget())); //505
    }
    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  -controller.getAnalog(okapi::ControllerAnalog::rightX));

    pros::delay(10);
  }
}