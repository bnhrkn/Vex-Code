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
#include "project/ui.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

using namespace okapi::literals;
okapi::Controller controller;
auto logger = std::make_shared<okapi::Logger>(
    okapi::TimeUtilFactory::createDefault().getTimer(), "/ser/sout",
    okapi::Logger::LogLevel::debug);

auto drive =
    okapi::ChassisControllerBuilder()
        .withSensors(okapi::RotationSensor(18, true), okapi::RotationSensor(20),
                     okapi::RotationSensor(19))
        .withMotors({-1, -9}, {3, 8})
        .withDimensions({okapi::AbstractMotor::gearset::blue, (36.0 / 60.0)},
                        {{3.25_in, 14.625_in}, okapi::imev5BlueTPR})
        .withOdometry({{2.75_in, 9.0_in, 6.625_in, 2.75_in}, 360},
                      okapi::StateMode::FRAME_TRANSFORMATION)
        .withLogger(logger)
        .buildOdometry();

auto model{drive->getModel()};

auto encModel{
    std::static_pointer_cast<okapi::ThreeEncoderSkidSteerModel>(model)};

void on_center_button() {}

auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

void initialize() {
  // controller.clear();

  display.setPosition({0_in, 0_in, 0_deg});

  okapi::Logger::setDefaultLogger(logger);

  encModel->resetSensors();

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  pros::delay(500); //There is a race condition somewhere

  drive->setState({0_in, 0_in, 0_deg});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // auto model{drive->getModel()};

  pros::Task matchTimer{[&] {
    controller.rumble("-");       // Match start rumble
    pros::delay(90000);           // Delay until 1:30
    for (int i = 0; i < 5; i++) { // Rumble countdown from 1:30 to 1:34
      controller.rumble(".");
      pros::delay(1000);
    }
    controller.rumble("-"); // Rumble on endgame start at 1:35
  }};

  constexpr std::array<std::string_view, 4> driveModes = {"arcade", "curvature",
                                                          "tank", "vector"};
  std::atomic<std::size_t> driveMode = 0;

  pros::Task modeSwitcher{[&] {
    auto i = driveMode.load(); // For looping over driveModes
    okapi::ControllerButton btn(okapi::ControllerDigital::Y);

    controller.setText( // Set the initial drive mode text
        0, 0, std::string("mode: ").append(driveModes.at(driveMode.load())));

    while (true) {
      if (btn.isPressed()) {
        driveMode = i;
        controller.setText(0, 0,
                           std::string("mode: ")
                               .append(driveModes.at(driveMode.load()))
                               .append(std::string_view("          ")));
        if (i + 1 < driveModes.size()) {
          i++;
        } else {
          i = 0;
        }
        int delay = 600;
        for (int i = 0; i < delay && btn.isPressed(); i += 50) {
          pros::delay(50);
        }
      } else {
        // std::cout << driveModes.at(driveMode.load()) << "\n";
        pros::delay(50);
      }
    }
  }};

  pros::Task trigger{[&] { // Task for the disc shooter trigger
    constexpr int burstShots = 3;
    constexpr int shotDelay = 100;
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

  // drive->setState({0_in, 0_in, 0_deg});

  // model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  // auto encModel {
  // std::static_pointer_cast<okapi::ThreeEncoderSkidSteerModel>(model) };

  // drive->driveToPoint({3_ft, 0_ft});

  while (true) {

    switch (driveMode.load()) {
    case 1:
      model->curvature(controller.getAnalog(okapi::ControllerAnalog::leftY),
                       controller.getAnalog(okapi::ControllerAnalog::rightX));
      break;
    case 2:
      model->tank(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  controller.getAnalog(okapi::ControllerAnalog::rightY));
      break;
    case 3:
      model->driveVector(controller.getAnalog(okapi::ControllerAnalog::leftY),
                         controller.getAnalog(okapi::ControllerAnalog::rightX));
      break;
    default:
      model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                    controller.getAnalog(okapi::ControllerAnalog::rightX));
      break;
    }
    // drive->setState({0_in, 0_in, 0_deg});

    //std::cout << drive->getState().str(okapi::inch, okapi::degree) << "\n";
    display.setPosition(drive->getState());
    // for (auto value : encModel->getSensorVals()) {
    //   std::cout << value << " ";
    // }
    // std::cout << "\n";

    pros::delay(10);
  }
}