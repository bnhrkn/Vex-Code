#include "main.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_chart.h"
#include "display/lv_objx/lv_list.h"
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/rotarysensor/rotationSensor.hpp"
#include "project/ramsete.hpp"
#include "project/ui.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstddef>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

using namespace okapi::literals;

okapi::Controller controller;

constinit const auto gearset = okapi::AbstractMotor::gearset::blue;
constinit const auto encoderUnit = okapi::AbstractMotor::encoderUnits::counts;
constinit const auto maxRPM = okapi::toUnderlyingType(gearset);

std::shared_ptr<okapi::ThreeEncoderSkidSteerModel> model;

const auto odomScales =
    okapi::ChassisScales({2.75_in, 9.0_in, 6.625_in, 2.75_in}, 360);

const auto chassisScales =
    okapi::ChassisScales({3.25_in, 14.625_in}, okapi::imev5BlueTPR);

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

std::array<std::vector<squiggles::ProfilePoint>, 2> paths;

// auto autonMode = std::atomic<auton::AutonMode>(auton::AutonMode::disabled);

void on_center_button() {}

void initialize() {
  const auto constraints = squiggles::Constraints(1.5, 2.5, 0.5);
  auto generator = squiggles::SplineGenerator(
      constraints, std::make_shared<squiggles::TankModel>(14.625, constraints));

  auto makeMotor = [](int iport, bool ireversed) {
    return okapi::Motor(iport, ireversed, gearset, encoderUnit);
  };

  model = std::make_shared<okapi::ThreeEncoderSkidSteerModel>(
      std::make_shared<okapi::MotorGroup>(okapi::MotorGroup{
          makeMotor(1, false), makeMotor(3, true), makeMotor(4, false)}),
      std::make_shared<okapi::MotorGroup>(okapi::MotorGroup{
          makeMotor(5, true), makeMotor(6, false), makeMotor(7, true)}),

      std::make_shared<okapi::RotationSensor>(18, true),
      std::make_shared<okapi::RotationSensor>(20, false),
      std::make_shared<okapi::RotationSensor>(19), maxRPM, 12000);

  // display.setPosition({0_in, 0_in, 0_deg});

  model->resetSensors();

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  pros::delay(
      500); // There is a race condition somewhere, sensors fail to reset.

  paths[0] = std::vector<squiggles::ProfilePoint>();
  paths[1] = generator.generate({{0, 0, 0}, {0, 1, 0}});

  // drive->setState({0_in, 0_in, 0_deg});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  auto odometry = okapi::ThreeEncoderOdometry(
      okapi::TimeUtilFactory::createDefault(), model, odomScales);

  std::vector<squiggles::ProfilePoint> &path = paths[0];
  try {
    path = paths.at(display.getAuton());
  } catch (const std::out_of_range &e) {
    path = paths[0];
  }

  for (auto iter = path.cbegin(); iter < path.cend(); ++iter) {
    odometry.step();
    const auto speeds =
        ramsete(stateToPose(odometry.getState()), *iter, 2.0, 0.7);
    const auto [left, right] = chassisToTankSpeeds(speeds, chassisScales);
    model->tank(left.convert(1_rpm), right.convert(1_rpm));
    if (iter++ != path.cend()) {
      pros::delay(iter++->time * 1000 - iter->time * 1000);
    }
  }
}

void opcontrol() {
  auto odometry = okapi::ThreeEncoderOdometry(
      okapi::TimeUtilFactory::createDefault(), model, odomScales);
  // auto model{drive->getModel()};

  pros::Task matchTimer{[&] {
    okapi::ControllerButton expand(okapi::ControllerDigital::right);
    pros::ADIDigitalOut cylinder(3, false);
    controller.rumble("-"); // Match start rumble
    pros::delay(95000);     // Delay until 1:35
    controller.rumble("-"); // Rumble on endgame start at 1:35
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
    pros::Motor intake(-20);
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
    flywheel.move_velocity(600);
    okapi::ControllerButton up(okapi::ControllerDigital::X);
    okapi::ControllerButton down(okapi::ControllerDigital::B);
    pros::ADIDigitalOut cyl(2, false);
    bool high = true;
    while (true) {
      if (up.changedToPressed() && !high) {
        high = true;
        flywheel.move_velocity(600);
        cyl.set_value(false);
      } else if (down.changedToPressed() && high) {
        high = false;
        flywheel.move_velocity(600 * 0.7);
        cyl.set_value(true);
      }
      pros::delay(20);
    }
  });

  while (true) {
    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  -controller.getAnalog(okapi::ControllerAnalog::rightX));

    pros::delay(10);
  }
}