#include "main.h"
#include "pros/rtos.hpp"
#include <memory>

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

  while (true) {
    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  controller.getAnalog(okapi::ControllerAnalog::leftX));

    pros::delay(10);
  }
}