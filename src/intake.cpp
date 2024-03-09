#include <utility>

#include <algorithm>
#include "project/algorithms.hpp"
#include "project/intake.hpp"

Intake::Intake(pros::Motor motor,
               // std::shared_ptr<Catapult> cata,
               // std::shared_ptr<okapi::SkidSteerModel> model,
               pros::Optical optical,
               pros::Vision vision,
               pros::adi::DigitalIn liftSwitch)
    : motor(std::move(motor)),
      optical(optical),
      vision(std::move(vision)),
      liftSwitch(liftSwitch),
      internalTask([this]() { taskFunction(); }) {
  optical.set_led_pwm(0);
  motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
  motor.set_brake_mode(pros::MotorBrake::coast);
  vision.set_signature(1, &green_sig);
}
Intake::~Intake() {
  internalTask.remove();
  motor.move_voltage(0);
}

void Intake::taskFunction() {
  int ballHeldTime = 0;
  constexpr int extraRunTime = 750;
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
  int32_t area_threshold = 0;
  std::array<pros::vision_object, 10> objects{};
  vision.read_by_sig(0, 1, objects.size(), objects.data());
  int32_t total_green_area = 0;
  for (auto object : objects) {
    total_green_area += object.height * object.width;
  }
  if (total_green_area > 0) {
    std::cout << "Area" << total_green_area << "\n";
  }
  return total_green_area > area_threshold;
}
bool Intake::hasBall() {
  constexpr auto holdingProxThreshold = 200;
  // std::cout << sensor.get_proximity() << "\n";
  return seeBall() && optical.get_proximity() > holdingProxThreshold;
}
