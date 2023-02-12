#include "project/ramsete.hpp"
#include "geometry/pose.hpp"
#include "main.h"
#include <cmath>
#include <numbers>
using namespace okapi::literals;
// RamseteController::RamseteController(const double ib, const double izeta)
//     : b{ib}, zeta{izeta} {};

// std::pair<double, double>
// RamseteController::step(const squiggles::Pose &pose,
//                         const squiggles::ProfilePoint &goal) {
//   const squiggles::Pose goalPose = goal.vector.pose;
//   const squiggles::Pose globalError = {pose.x - goalPose.x, pose.y -
//   goalPose.y,
//                                        pose.yaw};
//   const squiggles::Pose localError = {
//       globalError.x * (std::cos(globalError.yaw) +
//       std::sin(globalError.yaw)), globalError.y * (std::cos(globalError.yaw)
//       - std::sin(globalError.yaw)), globalError.yaw};
//   const double rotationVel = goal.curvature * goal.vector.vel;
//   const double gain = 2 * zeta * std::sqrt(std::pow(rotationVel, 2)) +
//                       b * (std::pow(goal.vector.vel, 2));
//   const double outLinearVel =
//       goal.vector.vel * std::cos(localError.yaw) + gain * localError.x;
//   const double outRotationVel = [&]() {
//     if (localError.yaw != 0) {
//       return rotationVel + gain * localError.yaw +
//              (b * goal.vector.vel * std::sin(localError.yaw) * localError.y)
//              /
//                  localError.yaw;
//     } else {
//       return 0.0;
//     }
//   }();
//   return {outLinearVel, outRotationVel};
// }

ChassisSpeeds ramsete(const squiggles::Pose &nowPose,
                      const squiggles::ProfilePoint &goalPoint, const double b,
                      const double zeta) {
  using namespace okapi::literals;
  const auto goalPose = goalPoint.vector.pose;
  const auto globalError =
      squiggles::Pose{goalPose.x - nowPose.x, goalPose.y - nowPose.y,
                      goalPose.yaw - nowPose.yaw};
  const auto localError =
      squiggles::Pose{std::cos(nowPose.yaw) * globalError.x +
                          std::sin(nowPose.yaw) * globalError.y,
                      -std::sin(nowPose.yaw) * globalError.x +
                          std::cos(nowPose.yaw) * globalError.y,
                      globalError.yaw};
  const auto rotationVel = goalPoint.curvature * goalPoint.vector.vel;
  const auto kgain = 2 * zeta *
                     std::sqrt(std::pow(rotationVel, 2) +
                               b * std::pow(goalPoint.vector.vel, 2));

  const auto outLinearVel =
      goalPoint.vector.vel * std::cos(localError.yaw) + kgain * localError.x;
  const auto outRotationVel = [&]() {
    if (localError.yaw != 0) {
      return rotationVel + kgain * localError.yaw +
             (b * goalPoint.vector.vel * std::sin(localError.yaw) *
              localError.y) /
                 localError.yaw;
    } else {
      return 0.0;
    }
  }();
  std::cout << "Diff. L: " << outLinearVel - goalPoint.vector.vel << " Diff. R: " << outRotationVel - rotationVel << std::endl;
  //return {outLinearVel * okapi::mps, outRotationVel * okapi::radps};
  return {goalPoint.vector.vel * okapi::mps, rotationVel * okapi::radps};
}

auto stateToPose(const okapi::OdomState state) -> squiggles::Pose {
  return {state.x.convert(okapi::meter), // Transform theta for squiggles
          state.y.convert(okapi::meter), state.theta.convert(okapi::radian)};
}

auto chassisToTankSpeeds(const ChassisSpeeds speeds,
                         const okapi::ChassisScales scales,
                         const okapi::AbstractMotor::GearsetRatioPair ratio)
    -> TankSpeeds {
  using namespace okapi::literals;

  okapi::QAngularSpeed leftSpeed =
      // Linear velocity minus tangential velocity,
      (speeds.linearVel - (scales.wheelTrack / 2) *
                              (speeds.angularVel /
                               okapi::radian)) / // meters per second divided by
      (scales.wheelDiameter / 2) *
      okapi::radian / ratio.ratio; // meters per degree = wheel angular speed
                                   // (degrees per second)
  okapi::QAngularSpeed rightSpeed =
      (speeds.linearVel +
       (scales.wheelTrack / 2) * (speeds.angularVel / okapi::radian)) /
      (scales.wheelDiameter / 2) * okapi::radian / ratio.ratio;

  return {leftSpeed, rightSpeed};
}

auto getConvertedState(const std::shared_ptr<okapi::Odometry> &odometry)
    -> okapi::OdomState {
  return convertState(odometry->getState());
}

auto convertState(const okapi::OdomState &state) -> okapi::OdomState {
  return {state.y, state.x, 90_deg - state.theta};
}
