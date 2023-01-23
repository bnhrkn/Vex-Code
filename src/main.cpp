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

constinit const auto gearset = okapi::AbstractMotor::gearset::blue;
const auto ratio = okapi::AbstractMotor::GearsetRatioPair{gearset, 60.0 / 36.0};
constinit const auto encoderUnit = okapi::AbstractMotor::encoderUnits::counts;
constinit const auto maxRPM = okapi::toUnderlyingType(gearset);

std::shared_ptr<okapi::ThreeEncoderSkidSteerModel> model;

const auto odomScales =
    okapi::ChassisScales({2.75_in, 7.35_in, 4.50_in, 2.75_in}, 360);

const auto chassisScales =
    okapi::ChassisScales({3.25_in, 12.75_in}, okapi::imev5BlueTPR);

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

using SquigglesPath = std::vector<squiggles::ProfilePoint>;

std::array<SquigglesPath, 3> paths;

// auto autonMode = std::atomic<auton::AutonMode>(auton::AutonMode::disabled);

void runPath(std::shared_ptr<okapi::ThreeEncoderSkidSteerModel> model,
             okapi::ThreeEncoderOdometry &odometry, SquigglesPath &path) {
  auto now = pros::millis();
  // double prevTime = 0;
  for (auto &point : path) {
    // std::cout << "Current ms: " << now
    //           << "    Delta ms: " << point.time * 1000 - prevTime <<
    //           std::endl;

    pros::delay(10);
    odometry.step();
    const auto speeds =
        ramsete(stateToPose(odometry.getState()), point, 2.0, 0.7);
    const auto [left, right] =
        chassisToTankSpeeds(speeds, chassisScales, ratio);
    // model->tank(left.convert(1_rpm), right.convert(1_rpm));
    model->left(left.convert(600_rpm));
    model->right(right.convert(600_rpm));
    pros::Task::delay_until(&now, 10);
  }
  model->stop();
}

void on_center_button() {}

void initialize() {
  const auto constraints = squiggles::Constraints(1.5, 4.5, 9);
  auto generator = squiggles::SplineGenerator(
      constraints,
      std::make_shared<squiggles::TankModel>(
          chassisScales.wheelTrack.convert(1_m), constraints),
      0.01);

  auto makeMotor = [](int iport, bool ireversed) {
    return okapi::Motor(iport, ireversed, gearset, encoderUnit);
  };
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

  paths[0] = std::vector<squiggles::ProfilePoint>();
  paths[1] = generator.generate({{0.0, 0.0, 0}, {0.609, 0.0, 0.0}});
  paths[2] =
      generator.generate({{0.0, 0.0, 0.0}, {0.0, 0.0, -std::numbers::pi / 2}});
  for (auto point : paths[2]) {
    std::cout << point.vector.pose.x << ", " << point.vector.pose.y << ", "
              << point.vector.pose.yaw << ", " << point.time << ", "
              << point.vector.vel << std::endl;
  }

}

void disabled() { model->stop(); }

void competition_initialize() {}

void autonomous() {
  auto odometry = okapi::ThreeEncoderOdometry(
      okapi::TimeUtilFactory::createDefault(), model, odomScales);
  runPath(model, odometry, paths[2]);
  
}

void opcontrol() {
  using std::literals::string_literals::operator""s;
  auto odometry = okapi::ThreeEncoderOdometry(
      okapi::TimeUtilFactory::createDefault(), model, odomScales);
  // auto model{drive->getModel()};

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
    odometry.step();
    display.setPosition(odometry.getState());

    pros::delay(10);
  }
}