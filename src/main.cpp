#include "main.h"
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

std::shared_ptr<okapi::OdomChassisController> chassis;
std::shared_ptr<okapi::ChassisModel> model;

const auto odomScales =
    okapi::ChassisScales({2.80_in, 7.625_in, 4.50_in, 2.75_in}, 360);

const auto chassisScales =
    okapi::ChassisScales({3.25_in, 12.75_in}, okapi::imev5BlueTPR);

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

void on_center_button() {}

void initialize() {
  std::ranges::for_each(
      std::array{14, 7, 6},
      pros::c::rotation_reset_position); // reset all rotation sensors to 0
  chassis =
      okapi::ChassisControllerBuilder()
          .withMotors({1, -2, 3}, {-11, 12, -17})
          .withGains({0.001, 0, 0.0001}, // Distance controller gains
                     {0.001, 0, 0.0001}, // Turn controller gains
                     {0.001, 0, 0.0001})
          .withDimensions(okapi::AbstractMotor::GearsetRatioPair(
                              okapi::AbstractMotor::gearset::blue, 36 / 60.0),
                          chassisScales)
          .withSensors(okapi::RotationSensor(14, true),
                       okapi::RotationSensor(7, false),
                       okapi::RotationSensor(6, false))
          .withOdometry()
          .buildOdometry();

  chassis->setState({0_in,0_in,90_deg});
  model = chassis->getModel();

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  pros::delay(500); // Legacy? race condition?
}

void disabled() { model->stop(); }

void competition_initialize() {}

void autonomous() {
  chassis->turnToPoint({24_in, 24_in});
}

void opcontrol() {
  chassis->stop();
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
    constexpr int burstShots = 3;
    constexpr int shotDelay = 200;
    pros::ADIDigitalOut cylinder(1, false);
    okapi::ControllerButton trigger(okapi::ControllerDigital::R2);
    int count = 0;
    int burstCount = 0;
    while (true) {
      if (trigger.isPressed() && burstCount < burstShots) {
        controller.setText(1, 0, std::to_string(++count));
        burstCount++;
        cylinder.set_value(true);
        pros::delay(shotDelay);
        cylinder.set_value(false);
        pros::delay(shotDelay);
      } else {
        if (!trigger.isPressed()) {
          burstCount = 0;
        }
        pros::delay(10);
      }
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
    pros::Motor flywheel(8);
    flywheel.set_gearing(pros::E_MOTOR_GEAR_BLUE);
    flywheel.move_velocity(600 * 0.80); // 0.87 far shot normal 42 deg / 30 deg
    okapi::ControllerButton up(okapi::ControllerDigital::X);
    okapi::ControllerButton down(okapi::ControllerDigital::B);
    pros::ADIDigitalOut cyl(2, true);
    bool high = true;
    while (true) {
      if (up.changedToPressed() && !high) {
        high = true;
        flywheel.move_velocity(600 * 0.80);
        cyl.set_value(true);
      } else if (down.changedToPressed() && high) {
        high = false;
        flywheel.move_velocity(600 * 0.7);
        cyl.set_value(false);
      }
      pros::delay(20);
    }
  });

  while (true) {
    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  -controller.getAnalog(okapi::ControllerAnalog::rightX));
    display.setPosition(chassis->getState());

    pros::delay(10);
  }
}