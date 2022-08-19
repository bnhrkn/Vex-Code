#include "main.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_btn.h"
#include "display/lv_objx/lv_chart.h"
#include "display/lv_objx/lv_list.h"
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

auto drive =
    okapi::ChassisControllerBuilder()
        .withMotors({-1, -9}, {3, 8})
        .withDimensions({okapi::AbstractMotor::gearset::blue, (36.0 / 60.0)},
                        {{3.25_in, 14.5625_in}, okapi::imev5BlueTPR})
        .build();

void on_center_button() {}

// display display(lv_scr_act());
// auto baseScr = lv_scr_act();
// auto navList = lv_list_create(baseScr, NULL);

// auto graph = lv_chart_create(baseScr, NULL);

// lv_res_t graphBtn(lv_obj_t*){
//   lv_obj_set_hidden(graph, false);
//   // lv_obj_set_hidden(autonSelector, true);
//   return LV_RES_OK;
// }
// lv_res_t autonBtn(lv_obj_t*){
//   lv_obj_set_hidden(graph, true);
//   return LV_RES_OK;
// }

// static const char * autonMap[] = {"1", "2", ""};
// std::map<const char *, std::size_t> autonMap = {};

auto display = ui::getInstance();

void initialize() {
  // controller.clear();

  display->init();
  // screen is 480 x 240 pixels
  // lv_theme_set_current(lv_theme_night_init(266, &lv_font_dejavu_20));

  // lv_obj_set_size(navList, 100, 240);
  // lv_obj_align(navList, baseScr, LV_ALIGN_IN_TOP_LEFT, 0, 0);
  // lv_list_add(navList, NULL, "Graph", graphBtn);
  // lv_list_add(navList, NULL, "Auton", autonBtn);

  // lv_obj_set_hidden(graph, true);
  // lv_obj_set_size(graph, 380, 240);
  // lv_obj_align(graph, navList, LV_ALIGN_OUT_RIGHT_MID, 0, 0);

  // lv_btn_set_action(graphBtn, LV_BTN_ACTION_PR, [=]() {});

  // auto chart = lv_chart_create(graphScreen, NULL);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
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

  pros::Task modeSwitcher{[&] {
    auto i = driveMode.load(); // For looping over driveModes
    okapi::ControllerButton btn (okapi::ControllerDigital::Y);

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
        std::cout << driveModes.at(driveMode.load()) << "\n";
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