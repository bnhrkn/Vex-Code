#include "main.h"
#include <memory>
#include "project/CustomChassisController.hpp"
#include "project/algorithms.hpp"
#include "project/catapult.hpp"

using namespace okapi::literals;
std::shared_ptr<CustomChassisController> chassis;
std::shared_ptr<okapi::SkidSteerModel> model;
std::shared_ptr<CustomOdom> odometry;

// enum class IntakeMode { in = 1, out = 2, off = 0 };
// enum class CatapultMode { half = 0, down = 1, shoot = 2 };
enum class AutonRoutine {
  disabled = 0,
  close_quals = 1,
  far_quals = 2,
  close_finals = 3,
  far_finals = 4
};
std::atomic<AutonRoutine> autonSelection = AutonRoutine::disabled;

void on_center_button() {}

void initialize() {
  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      "/ser/sout",                   // Output to the PROS terminal
      okapi::Logger::LogLevel::warn  // Show errors and warnings
      ));

  const okapi::IterativePosPIDController::Gains turnGains = {0.00527, 0, 0};
  const okapi::IterativePosPIDController::Gains distanceGains = {1.7, 0, 0};
  const okapi::ChassisScales scales = {{4.125_in, 12.375_in},
                                       okapi::imev5GreenTPR};
  const okapi::AbstractMotor::GearsetRatioPair gearing = {
      okapi::AbstractMotor::gearset::green, 1};

  auto left = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{13, 14, -12});
  auto right = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{-16, -17, 18});
  model = std::make_shared<okapi::SkidSteerModel>(
      left, right, left->getEncoder(), right->getEncoder(), 200, 12000);
  model->setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  model->setGearing(okapi::AbstractMotor::gearset::green);
  model->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  auto imu = std::make_shared<pros::IMU>(5);
  imu->reset(true);

  odometry = std::make_shared<CustomOdom>(left->getEncoder(),
                                          right->getEncoder(), imu, scales);

  odometry->setState({0_in, 0_in, 0_deg});

  auto turnPID = std::make_shared<okapi::IterativePosPIDController>(
      turnGains, okapi::ConfigurableTimeUtilFactory(0.75, 0.5, 100_ms).create(),
      // std::make_unique<lowPassFilter>(20, 10)
      std::make_unique<okapi::AverageFilter<3>>());

  auto distancePID = std::make_shared<okapi::IterativePosPIDController>(
      distanceGains,
      okapi::ConfigurableTimeUtilFactory(0.01, 0.02, 100_ms).create(),
      // std::make_unique<lowPassFilter>(20, 10)
      std::make_unique<okapi::AverageFilter<3>>());

  chassis = std::make_shared<CustomChassisController>(
      model, turnPID, distancePID, odometry, scales, gearing);

  // Auto Selection

  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  std::array<std::string, 5> autonNames = {"None", "Quals Near", "Quals Far",
                                           "Finals Near", "Finals Far"};
  size_t index = 0;
  size_t prevIndex = 1000;
  std::cout << "initializing\n";

  while (true) {
    // Index into autonnames circularly
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      index = (index + 1) % autonNames.size();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      index = (index + autonNames.size() - 1) % autonNames.size();
    }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      autonSelection = static_cast<AutonRoutine>(index);
      std::cout << static_cast<int>(autonSelection.load()) << "\n";
      controller.print(2, 0, "(X)");
      break;
    }
    if (index != prevIndex) {
      controller.print(2, 0, ("    " + autonNames[index] + "    ").c_str());
    }
    prevIndex = index;

    pros::delay(100);
  }
}

void disabled() {  // Does NOT run when you plug in
  chassis->cancelMovement();
  model->stop();
}

void competition_initialize() {  // Does run when you plug in
}

void autonomous() {
  std::cout << "Beginning Autonomous\n";
  switch (autonSelection) {
    case AutonRoutine::disabled: {
      std::cout << "Auton is disabled\n";
      // Don't do anything
    }
    case AutonRoutine::close_quals: {
      std::cout << "Running Close Quals\n";

      // Removes the ball from the match load zone
    }
    case AutonRoutine::far_quals: {
      // Scores the ball in far goal
      std::cout << "Running Far Quals\n";
      odometry->setState({0_in, 0_in, 0_deg});
      chassis->driveToPoint({12_in, 0_in}, false);
    }
    case AutonRoutine::close_finals: {
      std::cout << "Running Near Finals\n";

      // Tries to score as many points as possible
    }
    case AutonRoutine::far_finals: {
      std::cout << "Running Far Finals\n";

      // Tries to score as many points as possible
    }
  }
}

void opcontrol() {
  auto left = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{13, 14, -12});
  auto right = std::make_shared<okapi::MotorGroup>(
      std::initializer_list<okapi::Motor>{-16, -17, 18});
  okapi::SkidSteerModel model(left, right, left->getEncoder(),
                              right->getEncoder(), 200, 12000);
  model.setGearing(okapi::AbstractMotor::gearset::green);
  model.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  okapi::Controller controller;
  pros::Controller pcontroller(pros::E_CONTROLLER_MASTER);
  std::atomic<double> velocity = 0;
  std::atomic_bool cataArmed = false;
  std::atomic_bool cataDown = false;

  pros::Task intakeTask([&]() {
    // okapi::Timer inBtnTime;
    // okapi::Timer pickupLimitTimer;
    // okapi::Timer outBtnTime;
    std::array<double, 3> possibleHues = {80, 15, 132};
    constexpr double ballHueTolerance = 15;
    okapi::Motor intake(9);
    intake.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    okapi::OpticalSensor innerOptic(20);
    okapi::OpticalSensor outerOptic(7);
    innerOptic.setLedPWM(100);
    outerOptic.setLedPWM(100);

    intake.setGearing(okapi::AbstractMotor::gearset::blue);
    okapi::ControllerButton inBtn(okapi::ControllerDigital::L1);
    okapi::ControllerButton outBtn(okapi::ControllerDigital::L2);

    auto hasBall = [&]() -> bool {
      auto hue = innerOptic.getHue();
      if (innerOptic.getProximity() == 0) {
        return false;
      }
      // std::cout << "Inner Prox: " << innerOptic.getProximity() << "\n";
      return std::ranges::any_of(possibleHues, [&](double possibleHue) -> bool {
        return std::abs(hue - possibleHue) < ballHueTolerance;
      });
    };
    auto seeBall = [&]() -> bool {
      auto hue = outerOptic.getHue();
      // std::cout << outerOptic.getProximity() << "\n";
      if (outerOptic.getProximity() == 0) {
        return false;
      }
      // std::cout << "Outer Prox: " << outerOptic.getProximity() << "\n";
      return std::ranges::any_of(possibleHues, [&](double possibleHue) -> bool {
        return std::abs(hue - possibleHue) < ballHueTolerance;
      });
    };

    while (true) {
      if (!cataDown) {
        intake.moveVoltage(0);
      } else if ((seeBall() && !hasBall()) || (hasBall() && cataArmed)) {
        intake.moveVoltage(12000);
      } else if (outBtn.isPressed()) {
        intake.moveVoltage(-12000);
      } else if (velocity < 0) {
        intake.moveVoltage(7500 * -velocity);
      } else {
        intake.moveVoltage(0);
      }
      pros::delay(10);
    }
  });

  pros::Task cataTask([&]() {
    pros::Rotation cataSens{4};
    // auto catapult = std::make_shared<okapi::Motor>(16);
    // catapult->setGearing(okapi::AbstractMotor::gearset::red);
    okapi::Motor cata(-6);
    okapi::ControllerButton armCata(okapi::ControllerDigital::R1);
    okapi::ControllerButton shootCata(okapi::ControllerDigital::R2);
    auto getAngleDeg = [&]() {
      return okapi::OdomMath::constrainAngle180(cataSens.get_position() /
                                                100.0 * 1_deg)
          .convert(1_deg);
    };
    while (true) {
      if (armCata.changedToPressed()) {
        while (getAngleDeg() < 58) {
          cata.moveVoltage(12000);
          pros::delay(1);
        }
        cata.moveVoltage(0);
        cataArmed = true;
      }
      if (shootCata.changedToPressed()) {
        cataArmed = false;
        cataDown = false;
        while (cataSens.get_velocity() > -100) {
          cata.moveVoltage(12000);
          pros::delay(10);
        }
        cata.moveVoltage(0);
        while (getAngleDeg() > 10) {
          // Wait for catapult to go up
          pros::delay(10);
        }
      } else {
        while (shootCata.isPressed()) {
          cata.moveVoltage(12000);
          pros::delay(10);
        }
      }

      if (std::abs(getAngleDeg() - 47) > 5) {
        while (getAngleDeg() < 47) {
          cata.moveVoltage(12000);
          pros::delay(10);
        }
        cataDown = true;
        cata.moveVoltage(0);
      }
      pros::delay(10);
    }
  });

  while (true) {
    auto state = odometry->getState();
    std::cout << std::format("Position: (X: {}, Y: {}, Theta: {})\n",
                             state.x.convert(1_in), state.y.convert(1_in),
                             state.theta.convert(1_deg));
    model.arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                 controller.getAnalog(okapi::ControllerAnalog::rightX));
    velocity = controller.getAnalog(okapi::ControllerAnalog::leftY);

    pros::delay(10);
  }
}
