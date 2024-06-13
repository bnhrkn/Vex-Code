#include <utility>

#include <algorithm>
#include <format>
#include "project/algorithms.hpp"
#include "project/geometry.hpp"
#include "project/intake.hpp"

Intake::Intake(pros::Motor motor, std::shared_ptr<Camera> camera)
    : motor(std::move(motor)),
      camera(camera),
      internalTask([this]() { taskFunction(); }) {
  motor.set_gearing(pros::E_MOTOR_GEAR_BLUE);
  motor.set_brake_mode(pros::MotorBrake::coast);

  camera->set_signature(Camera::Color::GREEN, green_sig);
  camera->set_signature(Camera::Color::RED, red_sig);
  camera->set_signature(Camera::Color::BLUE, blue_sig);
}
Intake::~Intake() {
  internalTask.remove();
  motor.move_voltage(0);
}

void Intake::taskFunction() {
  int ballHeldTime = 10;
  int reverseTime = 0;
  constexpr int extraRunTime = 0;
  while (pros::Task::notify_take(true, 0) == 0U) {
    if (manual) {
      // std::cout << "Manual mode\n";
      pros::delay(10);
      continue;
    }
    if (hasBall()) {
      ballHeldTime += 10;
    } else {
      ballHeldTime = 0;
    }

    if (seeBall() && (!hasBall() || ballHeldTime < extraRunTime)) {
      // std::cout << "Intaking\n";
      motor.move_voltage(12000);
    } else if (ballHeldTime > 0 && ballHeldTime < reverseTime) {
      motor.move_voltage(-6000);
    }

    else {
      motor.move_velocity(0);
    }
    // if (seeBall() && !hasBall()) {
    //   motor.move_voltage(12000);
    // } else {
    //   motor.move_voltage(0);
    // }
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
  constexpr auto areaThreshold = 30;
  int32_t totalArea = 0;
  auto objects = camera->read_objects(
      {Camera::Color::GREEN, Camera::Color::RED, Camera::Color::BLUE});
  // std::cout << "clear\n";

  for (auto object : objects) {
    auto intersection = geometry::intersection(object, see_box);
    if (intersection.has_value()) {
      // std::cout << std::format("{},{},{},{},#00ff00\n", object.left_coord,
      //                          object.top_coord, object.width,
      //                          object.height);
      totalArea += intersection.value().area();
    }
    // std::cout << "update\n";
    pros::delay(100);
  }

  return totalArea > areaThreshold;
}
bool Intake::hasBall() {
  constexpr auto holdingAreaThreshold = 10;
  int32_t totalArea = 0;
  auto objects = camera->read_objects(
      {Camera::Color::GREEN, Camera::Color::RED, Camera::Color::BLUE});
  // std::cout << "clear\n";
  for (auto object : objects) {
    // std::cout << std::format("{},{},{},{},#00ff00\n", object.left_coord,
    //                          object.top_coord, object.width, object.height);
    auto intersection = geometry::intersection(object, have_box);

    if (intersection.has_value()) {
      totalArea += intersection.value().area();
    }
  }

  // std::cout << "update\n";
  return totalArea > holdingAreaThreshold;
}
