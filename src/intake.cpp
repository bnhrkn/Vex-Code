#include <utility>

#include "project/algorithms.hpp"
#include "project/intake.hpp"

constexpr std::array<double, 3> possibleHues = {80, 15, 132};
constexpr double ballHueTolerance = 15;

Intake::Intake(pros::Motor motor,
               std::shared_ptr<Catapult> cata,
               std::shared_ptr<okapi::SkidSteerModel> model,
               pros::Optical outerSensor,
               pros::Optical innerSensor)
    : motor(std::move(motor)),
      outerSensor(outerSensor),
      innerSensor(innerSensor),
      internalTask([this]() { taskFunction(); }),
      cata(std::move(cata)),
      model(std::move(model)) {
  outerSensor.set_led_pwm(255);
  innerSensor.set_led_pwm(255);
  motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
}
Intake::~Intake() {
  internalTask.remove();
  motor.move_voltage(0);
}

void Intake::taskFunction() {
  while (pros::Task::notify_take(true, 10) == 0U) {
    if (manual) {
      std::cout << "Manual mode\n";
      continue;
    }
    auto forwardVel = model->getLeftSideMotor()->getActualVelocity() +
                      model->getRightSideMotor()->getActualVelocity() / 2.0;
    std::cout << forwardVel << "\n";
    if (!cata->isReady()) {
      motor.move_voltage(0);
    } else if ((seeBall() && !hasBall()) || (hasBall() && cata->isArmed())) {
      motor.move_voltage(12000);
    } else if (forwardVel < 0 && (hasBall() || seeBall())) {
      motor.move_voltage(7500.0 * -forwardVel / 200.0);
    } else {
      motor.move_voltage(0);
    }
    pros::delay(10);
  }
}

void Intake::setManualMode(bool _manual, int32_t voltage) {
  bool prevManual = manual;
  manual = _manual;
  if (manual) {
    motor.move_voltage(voltage);
  } else if (!manual && prevManual) {
    motor.move_voltage(0);
  }
}

void Intake::release() {
  setManualMode(true, 12000);
  pros::delay(250);
  setManualMode(false);
}

bool Intake::isSettled() {
  // return intakeMotor.is_stopped();
  return motor.get_actual_velocity() == 0;
}
void Intake::waitUntilSettled(uint32_t timeoutMillis) {
  auto now = pros::millis();
  while (!isSettled() && pros::millis() - now < timeoutMillis) {
    pros::delay(10);
  }
}

bool Intake::seeBall() {
  auto hue = outerSensor.get_hue();
  // std::cout << outerSensor.get_proximity() << "," << outerSensor.get_hue()
  //           << "\n";
  if (outerSensor.get_proximity() == 0) {
    return false;
  }
  // std::cout << "Outer Prox: " << outerOptic.getProximity() << "\n";
  return std::ranges::any_of(possibleHues, [&](double possibleHue) -> bool {
    // std::cout << std::abs(hue - possibleHue) << "\n";
    return std::abs(hue - possibleHue) < ballHueTolerance;
  });
}
bool Intake::hasBall() {
  auto hue = innerSensor.get_hue();
  if (innerSensor.get_proximity() == 0) {
    return false;
  }
  // std::cout << "Inner Prox: " << innerOptic.getProximity() << "\n";
  return std::ranges::any_of(possibleHues, [&](double possibleHue) -> bool {
    return std::abs(hue - possibleHue) < ballHueTolerance;
  });
}
