#include <utility>

#include <algorithm>
#include "project/algorithms.hpp"
#include "project/intake.hpp"

Intake::Intake(pros::Motor motor,
               // std::shared_ptr<Catapult> cata,
               // std::shared_ptr<okapi::SkidSteerModel> model,
               pros::Optical sensor,
               pros::adi::DigitalIn liftSwitch)
    : motor(std::move(motor)),
      sensor(sensor),
      liftSwitch(liftSwitch),
      internalTask([this]() { taskFunction(); }) {
  sensor.set_led_pwm(0);
  motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
  motor.set_brake_mode(pros::MotorBrake::coast);
}
Intake::~Intake() {
  internalTask.remove();
  motor.move_voltage(0);
}

void Intake::taskFunction() {
  int ballHeldTime = 0;
  constexpr int extraRunTime = 500;
  while (pros::Task::notify_take(true, 0) == 0U) {
    if (manual) {
      std::cout << "Manual mode\n";
      pros::delay(10);
      continue;
    }
    if (hasBall()) {
      ballHeldTime += 10;
    } else {
      ballHeldTime = 0;
    }

    if (seeBall() && (!hasBall() || ballHeldTime < extraRunTime)) {
      motor.move_voltage(12000);
    } else {
      motor.move_velocity(0);
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
  auto hue = sensor.get_hue();
  // std::cout << sensor.get_proximity() << "," << sensor.get_hue() << "\n";
  if (sensor.get_proximity() == 0) {
    return false;
  }
  return hue > 52.0 && hue < 90.0;
}
bool Intake::hasBall() {
  constexpr auto holdingProxThreshold = 200;
  // std::cout << sensor.get_proximity() << "\n";
  return seeBall() && sensor.get_proximity() > holdingProxThreshold;
}
