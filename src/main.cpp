#include "main.h"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controllerUtil.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <atomic>
#include <cstddef>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

using namespace okapi::literals;

auto drive =
    okapi::ChassisControllerBuilder()
        .withMotors({-1, -9}, {3, 8})
        .withDimensions({okapi::AbstractMotor::gearset::blue, (36.0 / 60.0)},
                        {{3.25_in, 14.5625_in}, okapi::imev5BlueTPR})
        .build();

void on_center_button() {}

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  okapi::Controller controller;

  auto model{drive->getModel()};

  pros::Task matchTimer{[&] {
    controller.rumble("-");       // Match start rumble
    pros::delay(90000);           // Delay until 1:30
    for (int i = 0; i < 5; i++) { // Rumble countdown from 1:30 to 1:34
      controller.rumble(".");
      pros::delay(1000);
    }
    controller.rumble("-"); // Rumble on endgame start at 1:35
  }};

  constexpr std::array<std::string_view, 3> driveModes = {"arcade", "curvature",
                                                          "tank"};
  std::atomic<std::size_t> driveMode = 0;

  pros::Task modeSwitcher{
      [&] { // Task which allows the driver to select different modes nicely
        auto i = driveMode.load();
        okapi::ControllerButton yBtn{okapi::ControllerDigital::Y};

        controller.setText( // Set the initial drive mode text
            0, 0,
            std::string("mode: ").append(driveModes.at(driveMode.load())));

        while (true) {
          if (yBtn.isPressed()) {
            driveMode = i;
            controller.setText(0, 0,
                               std::string("mode: ")
                                   .append(driveModes.at(driveMode))
                                   .append(std::string_view("          ")));
            if (i + 1 < driveModes.size()) {
              i++;
            } else {
              i = 0;
            }
            int delay = 600;
            for (int i = 0; i < delay && !yBtn.isPressed(); i += 50) {
              pros::delay(50);
            }
          } else {
            pros::delay(50);
          }
        }
      }};

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

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
    default:
      model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                    controller.getAnalog(okapi::ControllerAnalog::rightX));
      break;
    }

    pros::delay(10);
  }
}