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
#include "project/tracker.hpp"
#include "project/ui.hpp"
#include "project/wings.hpp"

using namespace okapi::literals;
using namespace mp_units;
std::shared_ptr<CustomChassisController> chassis;
std::shared_ptr<okapi::SkidSteerModel> model;
std::shared_ptr<Tracker> tracker;
std::shared_ptr<pros::Imu> imu;
std::shared_ptr<UI> ui;
std::shared_ptr<Catapult> catapult;
std::shared_ptr<Intake> intake;
std::shared_ptr<okapi::IterativePosPIDController> distancePID;
std::shared_ptr<okapi::IterativePosPIDController> turnPID;
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

  const okapi::IterativePosPIDController::Gains turnGains = {0.004, 0, 1e-06};
  const okapi::IterativePosPIDController::Gains distanceGains = {1.8, 0, 0};

  const okapi::ChassisScales scales = {
      // Degrees per inch is defined in the class!!
      // Must be tuned when ratio changes are made.
      {3.25_in, 10.3_in},
      360};  // TPR MUST BE 360 WHEN USING DEGREES
  const okapi::AbstractMotor::GearsetRatioPair gearing = {
      okapi::AbstractMotor::gearset::blue, 48.0 / 36};

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

  tracker = std::make_shared<Tracker>(
      std::make_unique<Odometry>(left->getEncoder(), right->getEncoder(), imu,
                                 scales),
      std::make_unique<pros::GPS>(9), AllianceColor::red);
  tracker->setState({0_in, 0_in, 0_deg});

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
      model, turnPID, distancePID, tracker, scales, gearing);

  catapult = std::make_shared<Catapult>(pros::Motor(-17), pros::Rotation(1),
                                        pros::Distance(10));

  intake = std::make_shared<Intake>(pros::Motor(-14), camera);
  front_wings = std::make_shared<Wings>(pros::adi::DigitalOut('H'),
                                        pros::adi::DigitalOut('G'));
  rear_wings = std::make_shared<Wings>(pros::adi::DigitalOut('F'),
                                       pros::adi::DigitalOut('E'));
  pros::Task([&]() {
    while (true) {
      // tracker->step();
      tracker->setColor(ui->isBlueTeam() ? AllianceColor::blue
                                         : AllianceColor::red);
      ui->setPosition(tracker->getState());
      pros::delay(10);
    }
  });
}

void disabled() {  // Does NOT run when you plug in
  chassis->cancelMovement();
  model->stop();
  intake->setManualMode(false);
}

void competition_initialize() {  // Does run when you plug in
}

void autonomous() {
  AllianceColor alliance =
      ui->isBlueTeam() ? AllianceColor::blue : AllianceColor::red;
  tracker->setColor(alliance);

  std::cout << "Beginning Autonomous\n";
  using namespace auton;
  auto chassisSettled = [&]() { return chassis->isSettled(); };
  switch (ui->getAuton()) {
    case AutonMode::disabled:
      std::cout << "Auton is disabled\n";
      //  Don't do anything
      break;

    case AutonMode::close_quals:
      std::cout << "Running Close Quals\n";
      tracker->setState({5_ft - 3.5_in, -3_ft + 4_in,
                         geometry::angleToPoint(
                             {0_ft, -2_ft}, {5_ft - 3.5_in, -3_ft + 4_in})});
      front_wings->toggleExtended(Wings::Wing::left, true);
      intake->release();
      chassis->driveDistance(47_in);
      pros::delay(250);
      front_wings->toggleExtended(Wings::Wing::left, false);
      // intake->setManualMode(true, 12000);
      chassis->waitUntilSettled();
      chassis->driveDistance(-4_in);
      chassis->waitUntilSettled();
      std::cout << "First move settled\n";
      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();
      // intake->setManualMode(false);
      std::cout << "Center turn settled\n";
      front_wings->toggleExtended(Wings::Wing::left, true);

      chassis->driveDistance(20_in);
      chassis->waitUntilSettled();
      std::cout << "Center push settled\n";
      chassis->driveDistance(-5_in);
      chassis->waitUntilSettled();
      front_wings->toggleExtended(Wings::Wing::left, false);

      // chassis->driveToPoint({4_ft, -4_ft}, true);
      chassis->driveToPoint({4_ft - 4_in, -4_ft + 3_in}, true);
      std::cout << "Home drive settled\n";
      // chassis->driveToPoint({3_ft + 3_in, -5_ft + 6_in}, true);
      // chassis->turnToAngle(30_deg);
      // chassis->waitUntilSettled();
      // chassis->moveRaw(-0.8, 750);
      // tracker->setState({2_ft + 7_in, -5_ft + 3_in, 0_deg});
      // chassis->moveRaw(0.5, 250);

      chassis->driveToPoint({4_ft + 8.5_in, -4_ft + 4_in});
      rear_wings->toggleExtended(Wings::Wing::right, true);
      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();
      chassis->driveDistance(12_in);
      chassis->waitUntilSettled();
      rear_wings->toggleExtended(Wings::Wing::right, false);
      chassis->driveToPoint({3_ft, -3_ft});
      chassis->turnToAngle(-90_deg);
      chassis->waitUntilSettled();
      chassis->moveRaw(-0.5, 1000);
      // chassis->driveToPoint({3.5_ft, -0.75_ft}, true);
      rear_wings->toggleExtended(Wings::Wing::left);
      // chassis->turnRaw(0.5, 750);

      // chassis->driveToPoint({5_ft, -9.5_in});

      break;
    case AutonMode::far_quals:
      std::cout << "Running Far Quals\n";
      tracker->setState(
          {4_ft + 7.5_in, 2_ft + 7_in,
           geometry::angleToPoint({0_ft, 2_ft}, {4_ft + 7.5_in, 2_ft + 7_in})});
      intake->release();
      // front_wings->toggleExtended(Wings::Wing::right, true);
      chassis->driveDistance(45_in);
      // intake->setManualMode(true, 12000);
      // pros::delay(250);
      // front_wings->toggleExtended(Wings::Wing::right, false);
      chassis->waitUntilSettled();
      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();
      intake->setManualMode(true, -12000);
      pros::delay(500);
      intake->setManualMode(false);

      // chassis->turnToPoint({0_ft, 0_ft});
      // chassis->waitUntilSettled();
      // intake->setManualMode(true, 12000);
      // chassis->driveDistance(15_in);
      // chassis->waitUntilSettled();
      chassis->driveToPoint({5.5_in, 1_ft});

      chassis->driveDistance(-6_in);
      chassis->waitUntilSettled();

      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();

      intake->setManualMode(true, -12000);
      pros::delay(500);
      intake->setManualMode(false);

      // chassis->turnToPoint({2_ft, 0_ft});
      // chassis->waitUntilSettled();
      // intake->setManualMode(true, 12000);
      // chassis->driveDistance(16_in);
      // chassis->waitUntilSettled();
      // chassis->driveToPoint({19_in, 1_ft});

      chassis->driveToPoint({4_ft + 4_in, 4_ft - 6_in});
      chassis->driveToPoint({1_ft, 2_ft});

      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();
      front_wings->toggleExtended(Wings::Wing::left, true);
      chassis->moveRaw(0.5, 1050);
      front_wings->toggleExtended(Wings::Wing::left, false);
      tracker->setState({tracker->getState().x, 4_ft - 4_in, 90_deg});
      chassis->driveDistance(-10_in);
      chassis->waitUntilSettled();
      chassis->driveToPoint({5_ft, 3_ft});
      chassis->driveToPoint({5_ft, 11_in});

      break;
    case AutonMode::close_finals:
      std::cout << "Running Near Finals\n";

      tracker->setState({4_ft + 10.5_in, -4_ft - 3_in, 35_deg});
      intake->release();
      chassis->moveRaw(-0.7, 1250);
      chassis->driveDistance(7_in);
      chassis->waitUntilSettled();
      chassis->moveRaw(0.5, 250);
      chassis->driveToPoint({4_ft + 4.5_in, -4_ft + 4_in});
      rear_wings->toggleExtended(Wings::Wing::right, true);
      chassis->turnToAngle(90_deg);
      chassis->waitUntilSettled();
      chassis->driveDistance(12_in);
      chassis->waitUntilSettled();
      rear_wings->toggleExtended(Wings::Wing::right, false);
      chassis->driveToPoint({5_ft, -9.5_in});

      break;
    case AutonMode::far_finals:
      std::cout << "Running Far Finals\n";
      tracker->setState({5_ft, 9.125_in, -83_deg});
      intake->release();
      pros::delay(500);
      chassis->driveDistance(-31.5_in);
      chassis->waitUntilSettled();
      rear_wings->toggleExtended(Wings::Wing::left, true);
      chassis->driveToPoint({4_ft + 3_in, 4_ft + 4_in}, true);
      chassis->turnToAngle(155_deg + 180_deg);
      chassis->waitUntilSettled();
      rear_wings->toggleExtended(Wings::Wing::left, false);

      chassis->moveRaw(-1, 750);
      // tracker->setState({2_ft + 7_in, tracker->getState().y, 0_deg});
      chassis->driveDistance(7_in);
      chassis->waitUntilSettled();
      chassis->turnToAngle(180_deg);
      chassis->waitUntilSettled();
      intake->setManualMode(true, -12000);
      chassis->moveRaw(1, 1000);
      intake->setManualMode(false);
      chassis->driveDistance(-7_in);
      chassis->waitUntilSettled();
      chassis->turnToAngle(260_deg);
      chassis->waitUntilSettled();
      chassis->driveDistance(49_in);
      chassis->waitUntilSettled();

      chassis->turnToAngle(105_deg);
      chassis->waitUntilSettled();
      intake->setManualMode(true, -12000);
      chassis->moveRaw(1, 1250);
      intake->setManualMode(false);

      chassis->driveDistance(-12_in);
      chassis->waitUntilSettled();
      chassis->turnToAngle(135_deg);
      chassis->waitUntilSettled();
      chassis->moveRaw(-0.7, 1250);

      rear_wings->toggleExtended(Wings::Wing::right, true);

      break;
    case AutonMode::prog_skills:
      std::cout << "Running Prog Skills\n";

      break;
  }
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

  rear_wings->toggleExtended(Wings::Wing::left, false);
  rear_wings->toggleExtended(Wings::Wing::right, false);
  front_wings->toggleExtended(Wings::Wing::left, false);
  front_wings->toggleExtended(Wings::Wing::right, false);

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
      // chassis->turnByAngle(90_deg);
      chassis->driveDistance(24_in * direction);
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
