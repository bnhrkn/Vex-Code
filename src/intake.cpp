#include <utility>

#include "project/algorithms.hpp"
#include "project/intake.hpp"

auto distanceToDiscNum(pros::Distance& distance) -> int {
  auto value = distance.get();
  if (value > 85) {
    return 0;
  }
  if (value > 70) {
    return 1;
  }
  if (value > 50) {
    return 2;
  }
  if (value > 25) {
    return 3;
  }
  return 4;  // Disc is passing, cannot hold 4
}

Intake::Intake(pros::Motor motor,
               pros::Optical rollerSensor,
               pros::Distance indexerSensor,
               hueRange targetColor,
               hueRange otherColor,
               okapi::ControllerButton reverseBtn)
    : targetColor(targetColor),
      otherColor(otherColor),
      intakeMotor(std::move(motor)),
      rollerSensor(rollerSensor),
      indexerSensor(std::move(indexerSensor)),
      reverseBtn(std::move(reverseBtn)),
      internalTask([this]() { taskFunction(); }) {
  rollerSensor.set_led_pwm(255);
  intakeMotor.set_gearing(pros::E_MOTOR_GEAR_GREEN);
}
Intake::~Intake() {
  internalTask.remove();
}

void Intake::taskFunction() {
  constexpr auto proxThreshold = 70;
  auto jamDebounce = debouncer(5.0, 100, 100.0);
  auto powerDebounce = debouncer(2.0, 100, 0.0);
  auto discDebounce = debouncer(int{0}, 150, distanceToDiscNum(indexerSensor));
  auto torqueFilter = okapi::AverageFilter<10>();
  auto dropoutDetector = DisconnectDetector();
  while (pros::Task::notify_take(true, 10) == 0U) {
    discDebounce.poll(distanceToDiscNum(indexerSensor));
    jamDebounce.poll(intakeMotor.get_efficiency());
    powerDebounce.poll(intakeMotor.get_power());
    torqueFilter.filter(intakeMotor.get_torque());

    if (shouldSkip) {
      continue;
    }

    if (dropoutDetector.changedToConnected()) {
      intakeMotor.move_velocity(100);
      pros::delay(50);
    }

    if (reverseBtn.isPressed()) {
      intakeMotor.move_velocity(-200);
      continue;
    }
    if (manual) {
      intakeMotor.move_velocity(static_cast<int>(enabled) * 200);
      continue;
    }

    auto hue = rollerSensor.get_hue();
    auto prox = rollerSensor.get_proximity();
    // std::cout << "Torque: " << intakeMotor.get_torque()
    //           << " Filtered: " << torqueFilter.getOutput() << std::endl;
    if (jamDebounce.get() < 10 && intakeMotor.get_torque() > 1.0) {
      intakeMotor.move_velocity(-200);
      pros::delay(200);
    } else if (prox >= proxThreshold && inRange(hue, targetColor) &&
               !pros::competition::is_autonomous()) {
      intakeMotor.move_velocity(200);
    } else if (prox >= proxThreshold && inRange(hue, otherColor) &&
               (pros::competition::is_autonomous() == 0U)) {
      intakeMotor.move_velocity(200);
    } else if (discDebounce.get() < 3 || powerDebounce.get() > 4.5) {
      intakeMotor.move_velocity(200);
    } else {
      intakeMotor.move_velocity(0);
    }
  }
}

void Intake::toggleEnabledMode() {
  setEnabledMode(!enabled);
}

void Intake::toggleManualMode() {
  setManualMode(!manual);
}
void Intake::setManualMode(bool manual) {
  Intake::manual = manual;
}
void Intake::setEnabledMode(bool enabled) {
  Intake::enabled = enabled;
}
void Intake::setLoopSkip(bool shouldSkip) {
  Intake::shouldSkip = shouldSkip;
};

// Positive values flip roller upwards
void Intake::flipRaw(okapi::QAngle amount, std::uint32_t timeoutMillis) {
  auto start = pros::millis();
  internalTask.suspend();
  // while (internalTask.get_state() != pros::E_TASK_STATE_SUSPENDED)
  //   pros::delay(10);
  intakeMotor.move_velocity(0);
  intakeMotor.move_relative(
      amount.convert(okapi::degree) / ((36.0 / 84.0) * (12.0 / 24.0)), 200);
  while (std::abs(intakeMotor.get_target_position() -
                  intakeMotor.get_position()) > 5 &&
         start - pros::millis() <= timeoutMillis) {
    pros::delay(10);
  }
  internalTask.resume();
}

bool Intake::isSettled() {
  // return intakeMotor.is_stopped();
  return intakeMotor.get_actual_velocity() == 0;
}
void Intake::waitUntilSettled(uint32_t timeoutMillis) {
  auto now = pros::millis();
  while (!isSettled() && pros::millis() - now < timeoutMillis) {
    pros::delay(10);
  }
}
