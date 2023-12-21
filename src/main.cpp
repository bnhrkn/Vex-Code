#define EIGEN_DONT_VECTORIZE
#include "main.h"
#include <memory>
#include "Eigen/Core"
#include "project/CustomChassisController.hpp"
#include "project/algorithms.hpp"
#include "project/catapult.hpp"
#include "project/intake.hpp"
#include "project/pidTuner.hpp"
#include "project/ui.hpp"

using namespace okapi::literals;
std::shared_ptr<CustomChassisController> chassis;
std::shared_ptr<okapi::SkidSteerModel> model;
std::shared_ptr<CustomOdom> odometry;
std::shared_ptr<pros::Imu> imu;
std::shared_ptr<UI> ui;
std::shared_ptr<Catapult> catapult;
std::shared_ptr<Intake> intake;
std::shared_ptr<okapi::IterativePosPIDController> distancePID;
std::shared_ptr<okapi::IterativePosPIDController> turnPID;

void on_center_button() {}

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      "/ser/sout",                   // Output to the PROS terminal
      okapi::Logger::LogLevel::warn  // Show errors and warnings
      ));

  lvgl_init();
  ui = std::make_shared<UI>(std::unique_ptr<lv_obj_t>(lv_scr_act()));

  const okapi::IterativePosPIDController::Gains turnGains = {0.009, 0, 1e-06};
  const okapi::IterativePosPIDController::Gains distanceGains = {4.5, 0, 0};
  const okapi::ChassisScales scales = {
      {4.125_in, 12.375_in}, 360};  // TPR MUST BE 360 WHEN USING DEGREES
  const okapi::AbstractMotor::GearsetRatioPair gearing = {
      okapi::AbstractMotor::gearset::green, 1};

  auto left = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{13, -14, -12});
  auto right = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{16, -17, 18});
  model = std::make_shared<okapi::SkidSteerModel>(
      left, right, left->getEncoder(), right->getEncoder(), 200, 12000);
  model->setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  model->setGearing(okapi::AbstractMotor::gearset::green);
  model->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  imu = std::make_shared<pros::IMU>(1);
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

  catapult = std::make_shared<Catapult>(pros::Motor(-7), pros::Rotation(20));
  intake = std::make_shared<Intake>(pros::Motor(2), catapult, model,
                                    pros::Optical(15), pros::Optical(6));
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
      chassis->turnByAngle(720_deg);
      // Don't do anything
      break;

    case AutonMode::close_quals:
      std::cout << "Running Close Quals\n";
      odometry->setState({24_in - 5_in, 24_in - 5_in, 45_deg});
      intake->release();
      pros::delay(300);
      chassis->driveToPoint({3_ft, 3_ft});
      chassis->driveToPoint({5.5_ft, 30_in});

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
      chassis->driveToPoint({3_ft, 5.5_ft});

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
  }
}

void opcontrol() {
  // #define tuning
  okapi::Controller controller;
  okapi::ControllerButton armCata{okapi::ControllerDigital::R1};
  okapi::ControllerButton shootcata{okapi::ControllerDigital::R2};
  okapi::ControllerButton reverseIntake{okapi::ControllerDigital::L2};
  okapi::ControllerButton autoFire(okapi::ControllerDigital::A);
  bool autofiring = false;

#ifdef tuning
  constexpr auto size = 3;
  auto getter = [=]() -> std::array<double, size> {
    auto [P, I, D, B] = distancePID->getGains();
    return std::array{P, I, D};
  };
  auto setter = [=](std::array<double, 3> gains) -> void {
    distancePID->setGains({gains[0], gains[1], gains[2]});
    std::cout << "P" << std::setprecision(6) << gains[0] << "\nI"
              << std::setprecision(6) << gains[1] << "\nD"
              << std::setprecision(6) << gains[2] << "\n";
  };

  auto pidTuner = PIDConstantsTuner<3, getter, setter>();
  int direction = 1;

#endif

  while (true) {
#ifndef tuning
    auto state = odometry->getState();
    ui->setPosition(state);
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
    } else {
      intake->setManualMode(false);
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
      chassis->driveDistance(24_in * direction);
      auto startTime = pros::millis();
      // chassis->turnByAngle(90_deg);
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
