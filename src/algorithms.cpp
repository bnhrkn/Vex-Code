#include "project/algorithms.hpp"
#include <cmath>
#include <numbers>
#include "geometry/pose.hpp"
#include "main.h"

using namespace okapi::literals;

ChassisSpeeds ramsete(const squiggles::Pose& nowPose,
                      const squiggles::ProfilePoint& goalPoint,
                      const double b,
                      const double zeta) {
  using namespace okapi::literals;
  const auto goalPose = goalPoint.vector.pose;
  const auto globalError =
      squiggles::Pose{goalPose.x - nowPose.x, goalPose.y - nowPose.y,
                      goalPose.yaw - nowPose.yaw};
  const auto localError =
      squiggles::Pose{std::cos(-nowPose.yaw) * globalError.x -
                          std::sin(-nowPose.yaw) * globalError.y,
                      std::sin(-nowPose.yaw) * globalError.x +
                          std::cos(-nowPose.yaw) * globalError.y,
                      globalError.yaw};
  std::cout << std::format("Local error ({}, {}, {})", localError.x,
                           localError.y, localError.yaw);
  const auto rotationVel = goalPoint.curvature * goalPoint.vector.vel;
  const auto kgain = 2 * zeta *
                     std::sqrt(std::pow(rotationVel, 2) +
                               b * std::pow(goalPoint.vector.vel, 2));

  const auto outLinearVel =
      goalPoint.vector.vel * std::cos(localError.yaw) + kgain * localError.x;
  const auto outRotationVel =
      rotationVel + kgain * localError.yaw +
      sinc(localError.yaw) * b * goalPoint.vector.vel * localError.y;

  // std::cout << "Diff. L: " << outLinearVel - goalPoint.vector.vel
  //           << " Diff. R: " << outRotationVel - rotationVel << std::endl;
  return {outLinearVel * okapi::mps, outRotationVel * okapi::radps};
  // return {goalPoint.vector.vel * okapi::mps, rotationVel * okapi::radps};
}

auto stateToPose(const okapi::OdomState state) -> squiggles::Pose {
  return {state.x.convert(okapi::meter),  // Transform theta for squiggles
          state.y.convert(okapi::meter), state.theta.convert(okapi::radian)};
}
auto stateToPoint(const okapi::OdomState state) -> okapi::Point {
  return {state.x, state.y};
}

auto chassisToTankSpeeds(const ChassisSpeeds speeds,
                         const okapi::ChassisScales scales,
                         const okapi::AbstractMotor::GearsetRatioPair ratio)
    -> TankSpeeds {
  using namespace okapi::literals;

  okapi::QAngularSpeed leftSpeed =
      // Linear velocity minus tangential velocity,
      (speeds.linearVel -
       (scales.wheelTrack / 2) *
           (speeds.angularVel /
            okapi::radian)) /  // meters per second divided by
      (scales.wheelDiameter / 2) *
      okapi::radian / ratio.ratio;  // meters per degree = wheel angular speed
                                    // (degrees per second)
  okapi::QAngularSpeed rightSpeed =
      (speeds.linearVel +
       (scales.wheelTrack / 2) * (speeds.angularVel / okapi::radian)) /
      (scales.wheelDiameter / 2) * okapi::radian / ratio.ratio;

  return {leftSpeed, rightSpeed};
}

auto distanceCalcRPM(const okapi::QLength& distance) -> okapi::QAngularSpeed {
  auto d = distance.convert(1_in) + 0;
  // 350 + 0.78x + 5.86E-03x^2
  return (350 + 0.78 * d + 5.86E-3 * std::pow(d, 2)) * okapi::rpm;
}

lowPassFilter::lowPassFilter(double cutoffFreq, double deltaTime)
    : ePow(1 -
           std::exp(-deltaTime / 1000 * 2 * std::numbers::pi * cutoffFreq)){};
double lowPassFilter::filter(double value) {
  return output += (value - output) * ePow;
}
double lowPassFilter::getOutput() const {
  return output;
}

bool DisconnectDetector::changedToConnected() {
  auto value = !prevConnected && connected();
  prevConnected = connected();
  return value;
}
bool DisconnectDetector::connected() {
  return pros::c::controller_is_connected(pros::E_CONTROLLER_MASTER) != 0;
}
double sinc(double radians) {
  if (radians < 1E-6) {
    return 1.0 - radians * radians / 6;
  }
  return sin(radians) / radians;
}

template <typename T>
  requires std::is_signed_v<T> && requires(T a, T b) {
    { a < b } -> std::convertible_to<bool>;
  }
T abs(T value) {
  return (value < 0) ? -value : value;
}

constexpr bool approximatelyEqual(double a, double b, double epsilon) {
  return abs(a - b) < epsilon;
}