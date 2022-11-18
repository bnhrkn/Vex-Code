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
// #include "project/auton.hpp"
// #include "project/ramsete.hpp"
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

// auto display = ui(std::unique_ptr<lv_obj_t>(lv_scr_act()));

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
      std::make_shared<okapi::MotorGroup>(
          okapi::MotorGroup{makeMotor(1, true), makeMotor(9, true)}),
      std::make_shared<okapi::MotorGroup>(
          okapi::MotorGroup{makeMotor(3, false), makeMotor(8, false)}),

      std::make_shared<okapi::RotationSensor>(18, true),
      std::make_shared<okapi::RotationSensor>(20, false),
      std::make_shared<okapi::RotationSensor>(19), maxRPM, 12000);

  // display.setPosition({0_in, 0_in, 0_deg});

  model->resetSensors();

  model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  pros::delay(
      500); // There is a race condition somewhere, sensors fail to reset.

  paths[0] = generator.generate({{0, 0, 0}, {1, 0, 0}});
  paths[1] = generator.generate({{0, 0, 0}, {0, 1, 0}});

  // drive->setState({0_in, 0_in, 0_deg});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // auto odometry = okapi::ThreeEncoderOdometry(
  //     okapi::TimeUtilFactory::createDefault(), model, odomScales);

  // std::vector<squiggles::ProfilePoint> &path = paths[0];
  // switch (autonMode.load()) {
  //   using enum auton::AutonMode;
  // case auton1:
  //   path = paths[1];
  //   break;
  // default:
  //   break;
  // }

  // for (auto point : path){
  //   odometry.step();

  //   model->driveVector(double iySpeed, double izRotation)
  // }
}

void opcontrol() {
  auto odometry = okapi::ThreeEncoderOdometry(
      okapi::TimeUtilFactory::createDefault(), model, odomScales);
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
    pros::Motor intake(16);
    okapi::ControllerButton intakeBtn(okapi::ControllerDigital::L2);
    okapi::ControllerButton rollerBtn(okapi::ControllerDigital::L1);
    okapi::ControllerButton reverse(okapi::ControllerDigital::left);
    intake.set_gearing(pros::E_MOTOR_GEAR_BLUE);

    enum class intakeMode {
      fast,
      slow,
      off
    };
    auto mode = intakeMode::fast;
    while (true) {
      if(intakeBtn.changedToPressed()){
        if(mode == intakeMode::fast){
          mode = intakeMode::off;
        }
        else {
          mode = intakeMode::fast;
        }
      }
      else if (rollerBtn.changedToPressed()){
        if(mode == intakeMode::slow){
          mode = intakeMode::off;
        }
        else {
          mode = intakeMode::slow;
        }
      }

      if(reverse.isPressed()){
        intake.move_velocity(-600);
      }
      else{
        switch(mode){
          case intakeMode::fast: intake.move_velocity(600); break;
          case intakeMode::slow: intake.move_velocity(200); break;
          default: intake.move_velocity(0);
        }
      }
      pros::delay(20);
      
    }
  }};

  pros::Task tilter ([=]{
    
  });

  // drive->setState({0_in, 0_in, 0_deg});

  // model->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  // auto encModel {
  // std::static_pointer_cast<okapi::ThreeEncoderSkidSteerModel>(model) };

  // drive->driveToPoint({3_ft, 0_ft});

  // auto csvfile = std::ofstream("/usd/data.csv", std::ios::out);
  // const auto startTime = pros::millis();

  // auto killBtn = okapi::ControllerButton(okapi::ControllerDigital::X);

  // while (!killBtn.changedToPressed()) {

  pros::Motor flywheel(4);
  flywheel.move_voltage(12000);

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
    pros::delay(10);
  }
}