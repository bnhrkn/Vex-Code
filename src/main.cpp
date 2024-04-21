#include "project/MPChassisController.hpp"
#include "project/motionProfiling.hpp"
#define EIGEN_DONT_VECTORIZE
#include <memory>
#include "Eigen/Core"
#include "main.h"
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
// std::shared_ptr<CustomChassisController> chassis;
std::shared_ptr<MPChassisController> mpChassis;
std::shared_ptr<okapi::SkidSteerModel> model;
// std::shared_ptr<Odometry> odometry;
std::shared_ptr<Tracker> tracker;
std::shared_ptr<pros::Imu> imu;
std::shared_ptr<UI> ui;
std::shared_ptr<Catapult> catapult;
std::shared_ptr<Intake> intake;
// std::shared_ptr<okapi::IterativePosPIDController> distancePID;
// std::shared_ptr<okapi::IterativePosPIDController> turnPID;
std::shared_ptr<Wings> front_wings;
std::shared_ptr<Wings> rear_wings;
std::shared_ptr<Camera> camera;

void on_center_button() {}

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      "/ser/sout",                   // Output to the PROS terminal
      okapi::Logger::LogLevel::warn  // Show errors and warnings
      ));

  lvgl_init();
  camera = std::make_shared<Camera>(pros::Vision(8));
  camera->set_exposure(50);

  ui = std::make_shared<UI>(std::unique_ptr<lv_obj_t>(lv_scr_act()));

  // const okapi::IterativePosPIDController::Gains turnGains = {0.0055, 0,
  // 1e-06}; const okapi::IterativePosPIDController::Gains distanceGains = {2.1,
  // 0, 0};

  const DriveInfo driveInfo = {
      3.25_in,
      10.3_in,
      0.75,
      MotorColor::Blue,
  };
  const Constraints constraints = {driveInfo.maxSpeed(), 0.7_G,
                                   driveInfo.maxAngSpeed(),
                                   12 * 1_rad / 1_s / 1_s};

  // const Constraints constraints = {
  //     6 * okapi::foot / okapi::second, 0.5_G,
  //     6 * okapi::foot / okapi::second / scales.wheelTrack * okapi::radian,
  //     1.25_G / scales.wheelTrack / 2 * okapi::radian};

  const okapi::ChassisScales scales = {
      // Degrees per inch is defined in the class!!
      {driveInfo.wheelSize, driveInfo.trackWidth},
      360};  // TPR MUST BE 360 WHEN USING DEGREES
  // const okapi::AbstractMotor::GearsetRatioPair gearing = {
  // okapi::AbstractMotor::gearset::blue, 48.0 / 36};

  auto left = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{-18, -19, -20});
  auto right = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{11, 12, 13});
  model = std::make_shared<okapi::SkidSteerModel>(
      left, right, left->getEncoder(), right->getEncoder(), 600, 12000);
  model->setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  model->setGearing(okapi::AbstractMotor::gearset::blue);
  model->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  imu = std::make_shared<pros::IMU>(15);
  imu->reset(true);

  // odometry = std::make_shared<Odometry>(left->getEncoder(),
  // right->getEncoder(),
  //                                       imu, scales);

  // odometry->setState({0_in, 0_in, 0_deg});

  // turnPID = std::make_shared<okapi::IterativePosPIDController>(
  //     turnGains, okapi::ConfigurableTimeUtilFactory(1.6, 1.6,
  //     50_ms).create(),
  //     // std::make_unique<lowPassFilter>(20, 10)
  //     std::make_unique<okapi::AverageFilter<3>>());

  // distancePID = std::make_shared<okapi::IterativePosPIDController>(
  //     distanceGains,
  //     okapi::ConfigurableTimeUtilFactory(0.01, 0.02, 50_ms).create(),
  //     // std::make_unique<lowPassFilter>(20, 10)
  //     std::make_unique<okapi::AverageFilter<3>>());

  // chassis = std::make_shared<CustomChassisController>(
  //     model, turnPID, distancePID, odometry, scales, constraints, gearing);

  tracker = std::make_shared<Tracker>(
      std::make_unique<Odometry>(left->getEncoder(), right->getEncoder(), imu,
                                 scales),
      std::make_unique<pros::GPS>(7), AllianceColor::red);
  tracker->setState({0_in, 0_in, 0_deg});
  mpChassis = std::make_shared<MPChassisController>(model, tracker, driveInfo,
                                                    constraints);

  catapult = std::make_shared<Catapult>(pros::Motor(-17), pros::Rotation(1),
                                        pros::Distance(10));

  intake = std::make_shared<Intake>(pros::Motor(-14), camera);
  front_wings = std::make_shared<Wings>(pros::adi::DigitalOut('H'),
                                        pros::adi::DigitalOut('G'));
  rear_wings = std::make_shared<Wings>(pros::adi::DigitalOut('E'),
                                       pros::adi::DigitalOut('F'));
  pros::Task([&]() {
    while (true) {
      tracker->step();
      ui->setPosition(tracker->getState());
      pros::delay(10);
    }
  });

  // pros::Task([&]() {
  //   while (true) {
  //     auto greenObjs = camera->read_objects({Camera::Color::GREEN});
  //     std::cout << "clear\n";
  //     for (auto obj : greenObjs) {
  //       std::cout << std::format("{},{},{},{},{}", obj.left_coord,
  //                                obj.top_coord, obj.width, obj.height,
  //                                "#00ff00")
  //                 << "\n";
  //     }
  //     std::cout << "update\n";
  //     pros::delay(200);
  //   }
  // });

  // auto startTS = pros::micros();
  // auto path = genLinearProfile({0_m, 0_m}, {0.25_m, 0_m}, 2_mps, 8_mps2,
  // 10_ms);
  // // auto path = genAngularProfile({0_m, 0_m}, 0_deg, 90_deg, 120_rpm,
  // // 240_rpm / 1_s, 10_ms);
  // std::cout << "End Generating Points in " << pros::micros() - startTS
  //           << "us\n";
  // std::cout << "Started Generating Points\n"
  //           << "x,y,theta,linear velocity,linear acceleration,angular "
  //              "velocity,angular acceleration\n";
  // for (auto point : path) {
  //   std::cout << std::format(
  //       "{},{},{},{},{},{},{}\n", point.pose.x.convert(1_m),
  //       point.pose.y.convert(1_m), point.pose.theta.convert(1_rad),
  //       point.linearVel.convert(1_mps), point.linearAccel.convert(1_mps2),
  //       point.angularVel.convert(okapi::radps),
  //       point.angularAccel.convert(okapi::radps / okapi::second));
  // }

  // auto path = genLinearProfile({0_m, 0_m}, {0.25_m, 0_m}, 2_mps, 8_mps2,
  // 10_ms); for (auto point : path) {
  //   auto speeds = ramsete(point.pose, point, 2, 0.7);
  //   std::cout << std::format("Linear: {}, Angular: {}\n",
  //                            speeds.linearVel.convert(okapi::mps),
  //                            speeds.angularVel.convert(okapi::radps));
  // }
}

void disabled() {  // Does NOT run when you plug in
  mpChassis->stop();
  model->stop();
  intake->setManualMode(false);
}

void competition_initialize() {  // Does run when you plug in
}

void autonomous() {
  switch (ui->getAuton()) {
    default:
      tracker->setState({4_ft, 4_ft, 0_deg});
      // mpChassis->moveTo(okapi::Point{0_ft, 0.5_ft});
      mpChassis->turnTo(90_deg);
      // mpChassis->moveTo(okapi::Point{1_ft, 0_ft});
      //  mpChassis->turnTo(90_deg);
      //   mpChassis->moveTo(okapi::Point{2_ft, 2_ft});
      break;
  }
  /*
  std::cout << "Beginning Autonomous\n";
  using namespace auton;
  auto chassisSettled = [&]() { return mpChassis->isSettled(); };
  switch (ui->getAuton()) {
    case AutonMode::disabled:
      std::cout << "Auton is disabled\n";
      //  Don't do anything
      break;

    case AutonMode::close_quals:
      std::cout << "Running Close Quals\n";
      intake->release();
      odometry->setState({-4_ft, -4_ft - 6.5_in, 180_deg});
      front_wings->toggleExtended(Wings::Wing::right, true);
      pros::delay(500);

      // chassis->turnByAngle(180_deg);
      // chassis->waitUntilSettled();
      // wings->toggleExtended(Wings::Wing::right, false);

      chassis->turnRaw(0.5, 1000);
      front_wings->toggleExtended(Wings::Wing::right, false);
      chassis->driveToPoint({-5_ft, -4_ft + 10_in});
      chassis->driveToPoint({-5_ft, -3_ft});
      intake->setManualMode(true, -12000);
      pros::delay(750);

      chassis->driveToPoint({-5_ft, -4_ft + 7.5_in}, true);
      intake->setManualMode(false);

      chassis->driveToPoint({-3_ft, -3_ft});
      front_wings->toggleExtended(Wings::Wing::right, true);
      chassis->driveToPoint({-8_in, -3_ft});
      chassis->turnRaw(0.3, 1000);
      break;
    case AutonMode::far_quals:
      // Scores the ball in far goal
      std::cout << "Running Far Quals\n";
      odometry->setState({17_in, 36_in, 0_deg});
      intake->release();
      chassis->driveToPoint({5.2_ft, 3_ft}, false, 1_in);
      waitFor(chassisSettled);
      chassis->turnToAngle(-90_deg);
      waitFor(chassisSettled);
      chassis->driveDistance(7_in);
      waitFor(chassisSettled);
      chassis->driveDistance(-7_in);
      waitFor(chassisSettled);
      chassis->driveToPoint({3_ft, 3_ft});
      front_wings->toggleExtended(Wings::Wing::left, true);
      chassis->driveToPoint({3_ft, 5.5_ft});
      chassis->turnRaw(-0.3, 1000);
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
      intake->setManualMode(true, 12000);
      pros::delay(750);
      intake->setManualMode(false);
      odometry->setState({-4_ft, -4_ft - 6.5_in, 180_deg});
      front_wings->toggleExtended(Wings::Wing::right, true);
      pros::delay(500);
      chassis->turnRaw(0.65, 800);
      chassis->turnToAngle(0_deg);
      waitFor(chassisSettled);
      front_wings->toggleExtended(Wings::Wing::right, false);

      intake->setManualMode(true, -12000);
      pros::delay(750);
      intake->setManualMode(false);

      chassis->driveToPoint({-2_ft, -2_ft});
      chassis->driveToPoint({-2_ft, -2_ft + 11_in});
      chassis->driveToPoint({-2_ft, -3_ft}, true);
      chassis->driveToPoint({-3_ft, -3_ft}, true);
      chassis->driveToPoint({-3_ft, -5_ft});
      chassis->turnToAngle(0_deg);
      waitFor(chassisSettled);
      intake->setManualMode(true, -12000);

      // intake->setManualMode(false);
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
      front_wings->toggleExtended(Wings::Wing::right);
      int startTime = pros::millis();
      // while (pros::millis() - startTime < 35'000) {
      //   catapult->fire();
      //   pros::delay(10);
      // }
      front_wings->toggleExtended(Wings::Wing::right);
      chassis->turnByAngle(-10_deg);
      waitFor(chassisSettled);
      chassis->moveRaw(-0.5, 750);
      odometry->setState({-5_ft - 3.5_in, -3_ft - 5_in, 0_deg});
      chassis->driveDistance(6_in);
      waitFor(chassisSettled);
      chassis->driveToPoint({-3_ft, -5_ft + 2.5_in});
      chassis->driveToPoint({3_ft, -5_ft});
      chassis->driveToPoint({3_ft, -3_ft}, true);
      chassis->driveToPoint({1_ft, -3_ft}, true);
      chassis->driveToPoint({1_ft, 0_ft}, true);
      chassis->turnToPoint({-4_ft, 0_ft});
      front_wings->toggleExtended(Wings::Wing::left);
      front_wings->toggleExtended(Wings::Wing::right);
      pros::delay(250);
      chassis->driveDistance(-3.6_ft);
      waitFor(chassisSettled);
      chassis->driveDistance(2_ft);
      waitFor(chassisSettled);

      break;
  }*/
}

void opcontrol() {
  // #define tuning
  okapi::Controller controller;
  okapi::ControllerButton reverseIntake{okapi::ControllerDigital::R1};
  okapi::ControllerButton intakeBtn{okapi::ControllerDigital::L1};
  okapi::ControllerButton alwaysArm(okapi::ControllerDigital::Y);
  okapi::ControllerButton leftFrontWing(okapi::ControllerDigital::L2);
  okapi::ControllerButton rightFrontWing(okapi::ControllerDigital::R2);
  okapi::ControllerButton leftBackWing(okapi::ControllerDigital::left);
  okapi::ControllerButton rightBackWing(okapi::ControllerDigital::A);
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

    // std::cout << std::format(
    //     "Linear Acceleration: {}, Angular Acceleration: {}\n",
    //     imu->get_accel().y, imu->get_gyro_rate().z);

    // if (armCata.changedToPressed()) {
    //   catapult->arm();
    // }
    // if (shootcata.changedToPressed()) {
    //   catapult->fire();
    // }
    if (alwaysArm.changedToPressed()) {
      catapult->setArmMode(!catapult->getArmMode());
    }
    // if (autofiring) {
    //   catapult->fire();
    // }
    if (reverseIntake.isPressed()) {
      intake->setManualMode(true, -12000);
    } else if (intakeBtn.isPressed()) {
      intake->setManualMode(true, 12000);
    } else {
      intake->setManualMode(false);
    }

    if (leftFrontWing.changedToPressed()) {
      front_wings->toggleExtended(Wings::Wing::left);
    }
    if (rightFrontWing.changedToPressed()) {
      front_wings->toggleExtended(Wings::Wing::right);
    }
    if (leftBackWing.changedToPressed()) {
      rear_wings->toggleExtended(Wings::Wing::left);
    }
    if (rightBackWing.changedToPressed()) {
      rear_wings->toggleExtended(Wings::Wing::right);
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
