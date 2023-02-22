#include "project/flywheel.hpp"
#include "project/algorithms.hpp"

using Gains = okapi::IterativeVelPIDController::Gains;

Flywheel::Flywheel(pros::Motor motor, Gains gains, SettledUtil settledUtil)
    : motor(motor), settledUtil(settledUtil),
      internalTask([=, this]() { taskFunc(); }), gains(gains) {}

Flywheel::~Flywheel() {
  internalTask.notify();
  pros::c::task_abort_delay((pros::task_t)internalTask);
  internalTask.join();
  motor.move_voltage(0);
}

bool Flywheel::isSettled() const {
  std::scoped_lock<pros::Mutex> lock(settledUtilMutex);
  return settledUtil.isSettled();
}

void Flywheel::waitUntilSettled(const std::uint32_t pollingRate) const {
  while (!isSettled()) {
    pros::delay(pollingRate);
  }
}

Gains Flywheel::getGains() const {
  std::scoped_lock<pros::Mutex> lock(gainsMutex);
  return gains;
}

void Flywheel::setGains(Gains values) {
  std::scoped_lock<pros::Mutex> lock(gainsMutex);
  gains = values;
}

okapi::QAngularSpeed Flywheel::getTarget() const {
    return target.load() * okapi::rpm;
}

void Flywheel::setTarget(okapi::QAngularSpeed speed) {

  setTarget(speed.convert(okapi::rpm));
}

void Flywheel::setTarget(double rpm) {
  target = rpm;
} // Need to also set prevalue to prevent derivative kick}

void Flywheel::taskFunc() {
  auto velFilter = lowPassFilter(5, 10);
  velFilter.filter(motor.get_actual_velocity());
  // auto prevValue = motor.get_actual_velocity();
  auto prevTime = pros::millis();
  auto prevTarget = target.load();
  auto prevError = 0.0;
  while (!pros::Task::notify_take(true, 0)) {
    auto value = velFilter.filter(motor.get_actual_velocity());
    // std::cout << "Filtered: " << value << "Real: " <<
    // motor.get_actual_velocity() << "\n";
    auto error = target - value;
    // printf("Running Loop\n");
    //  Eliminate derivative kick on setpoint change
    auto delta = target != prevTarget ? 0.0 : error - prevError;

    auto [kP, kD, kF, kSF] = [&]() {
      std::scoped_lock<pros::Mutex> lock(gainsMutex);
      return gains;
    }();

    // prevValue = value;
    prevError = error;
    prevTarget = target.load();

    auto output = (kP * error) - (kD * delta) + (kF * target) + kSF;
    if (target != 0) {
      motor.move_voltage(output);
    }
    else{
        motor.move_voltage(0);
    }
    {
      std::scoped_lock<pros::Mutex> lock(settledUtilMutex);
      settledUtil.step(value, target);
    }
    pros::Task::delay_until(&prevTime, 10);
  }
}