#define EIGEN_DONT_VECTORIZE
#include "main.h"
#include <memory>
#include "Eigen/Core"
// #include "gsl/gsl-lite.hpp"
#include "mp-units/core.h"
#include "project/CustomChassisController.hpp"
#include "project/algorithms.hpp"
#include "project/catapult.hpp"
#include "project/intake.hpp"
#include "project/pidTuner.hpp"
#include "project/ui.hpp"
#include "project/wings.hpp"

using namespace okapi::literals;
using namespace mp_units;
std::shared_ptr<CustomChassisController> chassis;
std::shared_ptr<okapi::SkidSteerModel> model;
std::shared_ptr<CustomOdom> odometry;
std::shared_ptr<pros::Imu> imu;
std::shared_ptr<UI> ui;
std::shared_ptr<Catapult> catapult;
std::shared_ptr<Intake> intake;
std::shared_ptr<okapi::IterativePosPIDController> distancePID;
std::shared_ptr<okapi::IterativePosPIDController> turnPID;
std::shared_ptr<Wings> wings;

void on_center_button() {}

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      "/ser/sout",                   // Output to the PROS terminal
      okapi::Logger::LogLevel::warn  // Show errors and warnings
      ));

  lvgl_init();
  ui = std::make_shared<UI>(std::unique_ptr<lv_obj_t>(lv_scr_act()));

  const okapi::IterativePosPIDController::Gains turnGains = {0.0055, 0, 1e-06};
  const okapi::IterativePosPIDController::Gains distanceGains = {2.1, 0, 0};
  const okapi::ChassisScales scales = {
      // Degrees per inch is defined in the class!!
      {2.75_in, 10.37_in},
      360};  // TPR MUST BE 360 WHEN USING DEGREES
  const okapi::AbstractMotor::GearsetRatioPair gearing = {
      okapi::AbstractMotor::gearset::blue, 48.0 / 36};

  auto left = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{-1, -2, -3});
  auto right = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{4, 5, 6});
  model = std::make_shared<okapi::SkidSteerModel>(
      left, right, left->getEncoder(), right->getEncoder(), 600, 12000);
  model->setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  model->setGearing(okapi::AbstractMotor::gearset::blue);
  model->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  imu = std::make_shared<pros::IMU>(18);
  imu->reset(true);

  odometry = std::make_shared<CustomOdom>(left->getEncoder(),
                                          right->getEncoder(), imu, scales);

  odometry->setState({0_in, 0_in, 0_deg});

  turnPID = std::make_shared<okapi::IterativePosPIDController>(
      turnGains, okapi::ConfigurableTimeUtilFactory(1.6, 1.6, 50_ms).create(),
      // std::make_unique<lowPassFilter>(20, 10)
      std::make_unique<okapi::AverageFilter<3>>());

  distancePID = std::make_shared<okapi::IterativePosPIDController>(
      distanceGains,
      okapi::ConfigurableTimeUtilFactory(0.01, 0.02, 50_ms).create(),
      // std::make_unique<lowPassFilter>(20, 10)
      std::make_unique<okapi::AverageFilter<3>>());

  chassis = std::make_shared<CustomChassisController>(
      model, turnPID, distancePID, odometry, scales, gearing);

  catapult = std::make_shared<Catapult>(pros::Motor(-12), pros::Rotation(16),
                                        pros::Distance(14));
  intake = std::make_shared<Intake>(pros::Motor(10), pros::Optical(11),
                                    pros::adi::DigitalIn(4));
  wings = std::make_shared<Wings>(pros::adi::DigitalOut(1),
                                  pros::adi::DigitalOut(2));
  pros::Task([&]() {
    while (true) {
      ui->setPosition(odometry->getState());
      pros::delay(10);
    }
  });
}

void disabled() {  // Does NOT run when you plug in
  chassis->cancelMovement();
  model->stop();
}

void competition_initialize() {  // Does run when you plug in
}

void autonomous() {
  std::cout << "Beginning Autonomous\n";
  using namespace auton;
  std::cout << static_cast<int>(ui->getAuton());
  switch (ui->getAuton()) {
    case AutonMode::disabled:
      std::cout << "Auton is disabled\n";
      chassis->moveRaw(-0.5, 500);
      //  Don't do anything
      break;

    case AutonMode::close_quals:
      std::cout << "Running Close Quals\n";
      // odometry->setState({24_in - 5_in, 24_in - 5_in, 45_deg});
      // intake->release();
      // pros::delay(300);
      // chassis->driveToPoint({3_ft, 3_ft});
      // chassis->driveToPoint({5.5_ft, 30_in});
      odometry->setState({-3_ft - 6.5_in, -4_ft - 6.25_in, 180_deg});
      wings->toggleExtended(Wings::Wing::right, true);
      chassis->turnToAngle(0_deg);
      chassis->waitUntilSettled();
      wings->toggleExtended(Wings::Wing::right, false);
      // chassis->turnToAngle(180_deg - 45_deg);
      // chassis->waitUntilSettled();
      // intake->setManualMode(true, -12000);
      // pros::delay(500);
      // intake->setManualMode(false);
      // chassis->driveToPoint({-5_ft, -4_ft}, true);
      // chassis->driveToPoint({-5_ft, -3_ft + 2.25_in}, true);
      chassis->driveToPoint({-3_ft, -3_ft});
      wings->toggleExtended(Wings::Wing::right, true);
      chassis->driveToPoint({-8_in, -3_ft});
      chassis->turnByAngle(90_deg);
      chassis->waitUntilSettled();

      // Removes the ball from the match load zone
      break;
    case AutonMode::far_quals:
      // Scores the ball in far goal
      std::cout << "Running Far Quals\n";
      odometry->setState({17_in, 36_in, 0_deg});
      intake->release();
      chassis->driveToPoint({5.2_ft, 3_ft}, false, 1_in);
      chassis->waitUntilSettled();
      chassis->turnToAngle(-90_deg);
      chassis->waitUntilSettled();
      chassis->driveDistance(7_in);
      chassis->waitUntilSettled();
      chassis->driveDistance(-7_in);
      chassis->waitUntilSettled();
      chassis->driveToPoint({3_ft, 3_ft});
      wings->toggleExtended(Wings::Wing::left, true);
      chassis->driveToPoint({3_ft, 5.5_ft});
      chassis->turnToAngle(45_deg);
      chassis->waitUntilSettled();
      // odometry->setState({1_ft - 1.5_in, -5_ft, 180_deg});
      // chassis->driveDistance(2_in);
      // chassis->waitUntilSettled();
      // pros::delay(250);
      // chassis->driveToPoint({3_ft, -5_ft}, true, 0_in, 0.5);
      // chassis->driveToPoint({5_ft, -3_ft}, true, 0_in, 0.5);
      // chassis->driveToPoint({5_ft, -2.75_ft}, true);  // Score 1st ball
      // chassis->driveToPoint({5_ft, -3_ft}, false);
      // chassis->driveToPoint({5_ft, -2.75_ft}, false);  // Score 2nd ball
      // chassis->driveToPoint({5_ft, -3_ft}, false);
      // chassis->driveToPoint({0.875_ft, -2.375_ft}, false);
      // chassis->turnToPoint({4_ft, -1_ft});
      // intake->setManualMode(true, -12000);
      // pros::delay(500);
      // intake->setManualMode(false);
      // chassis->driveToPoint({0.625_ft, -0.5_ft}, false);
      // wings->toggleExtended(Wings::Wing::left, true);
      // wings->toggleExtended(Wings::Wing::right, true);
      // chassis->driveToPoint({3.6_ft, -0.5_ft}, true);
      // wings->toggleExtended(Wings::Wing::left, false);
      // wings->toggleExtended(Wings::Wing::right, false);
      // chassis->driveToPoint({3_ft, -0.5_ft}, false);
      // chassis->driveToPoint({3.6_ft, -0.5_ft}, false);

      break;
    case AutonMode::close_finals:
      std::cout << "Running Near Finals\n";
      odometry->setState({24_in - 5_in, 24_in - 5_in, 45_deg});
      intake->release();
      pros::delay(300);

      chassis->driveToPoint({1.85_ft, 1.85_ft});

      chassis->driveToPoint({0.7_ft, 2.75_ft}, true);
      chassis->driveToPoint({0.7_ft, 3.125_ft}, true);
      chassis->driveToPoint({0.7_ft, 2.88_ft}, false);
      chassis->driveToPoint({3_ft, 1_ft}, true);
      chassis->driveToPoint({4.25_ft, 1_ft}, true);

      break;
    case AutonMode::far_finals:
      std::cout << "Running Far Finals\n";
      odometry->setState({17_in, 36_in, 0_deg});
      // pros::delay(100);
      intake->release();
      chassis->driveToPoint({5.25_ft, 3_ft}, false, 1_in);
      chassis->driveToPoint({5.25_ft, 2.3_ft});
      chassis->driveToPoint({5.25_ft, 3_ft}, true);

      chassis->driveToPoint({5.75_ft, 3.75_ft});
      chassis->driveToPoint({5.25_ft, 2.3_ft});
      chassis->driveToPoint({5.25_ft, 3_ft}, true);

      chassis->driveToPoint({4_ft, 5_ft});
      chassis->driveToPoint({5.25_ft, 2.3_ft});
      chassis->driveToPoint({5.25_ft, 3_ft}, true);

      break;
    case AutonMode::prog_skills:
      std::cout << "Running Prog Skills\n";
      odometry->setState({-3_ft - 5_in, -5_ft + 5.5_in, 0_deg});
      chassis->driveToPoint({-5_ft, -3_ft - 5_in}, true);
      chassis->turnToPoint({4_ft, -1_ft});
      // odometry->setState({-5_ft, -3_ft, 0_deg});
      wings->toggleExtended(Wings::Wing::right);
      int startTime = pros::millis();
      while (pros::millis() - startTime < 35'000) {
        catapult->fire();
        pros::delay(10);
      }
      wings->toggleExtended(Wings::Wing::right);
      chassis->turnByAngle(-10_deg);
      chassis->waitUntilSettled();
      chassis->moveRaw(-0.5, 750);
      odometry->setState({-5_ft - 3.5_in, -3_ft - 5_in, 0_deg});
      chassis->driveDistance(6_in);
      chassis->waitUntilSettled();
      chassis->driveToPoint({-3_ft, -5_ft + 2.5_in});
      chassis->driveToPoint({3_ft, -5_ft});
      chassis->driveToPoint({3_ft, -3_ft}, true);
      chassis->driveToPoint({1_ft, -3_ft}, true);
      chassis->driveToPoint({1_ft, 0_ft}, true);
      chassis->turnToPoint({-4_ft, 0_ft});
      wings->toggleExtended(Wings::Wing::left);
      wings->toggleExtended(Wings::Wing::right);
      pros::delay(250);
      chassis->driveDistance(-3.6_ft);
      chassis->waitUntilSettled();
      chassis->driveDistance(2_ft);
      chassis->waitUntilSettled();

      break;
  }
}

void opcontrol() {
  // #define tuning
  okapi::Controller controller;
  okapi::ControllerButton armCata{okapi::ControllerDigital::R1};
  okapi::ControllerButton shootcata{okapi::ControllerDigital::R2};
  okapi::ControllerButton reverseIntake{okapi::ControllerDigital::L2};
  okapi::ControllerButton intakeBtn{okapi::ControllerDigital::R2};
  okapi::ControllerButton autoFire(okapi::ControllerDigital::A);
  okapi::ControllerButton leftWing(okapi::ControllerDigital::L1);
  okapi::ControllerButton rightWing(okapi::ControllerDigital::R1);
  okapi::ControllerButton elevationMech(okapi::ControllerDigital::Y);
  bool autofiring = false;
  bool elevation = false;
  auto startTime = pros::millis();

#ifdef tuning
  constexpr auto size = 3;
  auto getter = [=]() -> std::array<double, size> {
    auto [P, I, D, B] = turnPID->getGains();
    return std::array{P, I, D};
  };
  auto setter = [=](std::array<double, 3> gains) -> void {
    turnPID->setGains({gains[0], gains[1], gains[2]});
    std::cout << "P" << std::setprecision(6) << gains[0] << "\nI"
              << std::setprecision(6) << gains[1] << "\nD"
              << std::setprecision(6) << gains[2] << "\n";
  };

  auto pidTuner = PIDConstantsTuner<3, getter, setter>();
  int direction = 1;

#endif

  while (true) {
#ifndef tuning

    model->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                  controller.getAnalog(okapi::ControllerAnalog::rightX));
    if (armCata.changedToPressed()) {
      catapult->arm();
    }
    if (shootcata.changedToPressed()) {
      catapult->fire();
    }
    if (autoFire.changedToPressed()) {
      autofiring = !autofiring;
    }
    if (autofiring) {
      catapult->fire();
    }
    if (reverseIntake.isPressed()) {
      intake->setManualMode(true, -12000);
    } else if (intakeBtn.isPressed()) {
      intake->setManualMode(true, 12000);
    } else {
      intake->setManualMode(false);
    }

    if (leftWing.changedToPressed()) {
      wings->toggleExtended(Wings::Wing::left);
    }
    if (rightWing.changedToPressed()) {
      wings->toggleExtended(Wings::Wing::right);
    }
    if (
        // pros::millis() - startTime > 90 * 1000 &&
        elevationMech.changedToPressed()) {
      elevation = !elevation;
      pros::adi::DigitalOut(3).set_value(elevation);
    }
#endif
#ifdef tuning
    if (controller[okapi::ControllerDigital::up].changedToPressed()) {
      pidTuner.inc();
    }
    if (controller[okapi::ControllerDigital::down].changedToPressed()) {
      pidTuner.dec();
    }
    if (controller[okapi::ControllerDigital::right].changedToPressed()) {
      pidTuner.decPresicion();
    }
    if (controller[okapi::ControllerDigital::left].changedToPressed()) {
      pidTuner.incPrecision();
    }
    if (controller[okapi::ControllerDigital::B].changedToPressed()) {
      pidTuner.nextGain();
    }
    if (controller[okapi::ControllerDigital::L1].changedToPressed()) {
      // chassis->driveDistance(24_in * direction);
      auto startTime = pros::millis();
      chassis->turnByAngle(90_deg);
      direction *= -1;
      while (!chassis->isSettled() &&
             !controller[okapi::ControllerDigital::L2].changedToPressed()) {
        pros::delay(10);
      }
      std::cout << "Settled in " << pros::millis() - startTime << "ms\n";
      chassis->cancelMovement();
      controller.rumble(".");
    };
#endif

    pros::delay(10);
  }
}
