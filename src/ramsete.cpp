#include "project/ramsete.hpp"
#include "geometry/pose.hpp"
#include <cmath>

RamseteController::RamseteController(const double ib, const double izeta)
    : b{ib}, zeta{izeta} {};

std::pair<double, double>
RamseteController::step(const squiggles::Pose &pose,
                        const squiggles::ProfilePoint &goal) {
  const squiggles::Pose goalPose = goal.vector.pose;
  const squiggles::Pose globalError = {pose.x - goalPose.x, pose.y - goalPose.y,
                                       pose.yaw};
  const squiggles::Pose localError = {
      globalError.x * (std::cos(globalError.yaw) + std::sin(globalError.yaw)),
      globalError.y * (std::cos(globalError.yaw) - std::sin(globalError.yaw)),
      globalError.yaw};
  const double rotationVel = goal.curvature * goal.vector.vel;
  const double gain = 2 * zeta * std::sqrt(std::pow(rotationVel, 2)) +
                      b * (std::pow(goal.vector.vel, 2));
  const double outLinearVel =
      goal.vector.vel * std::cos(localError.yaw) + gain * localError.x;
  const double outRotationVel = [&]() {
    if (localError.yaw != 0) {
      return rotationVel + gain * localError.yaw +
             (b * goal.vector.vel * std::sin(localError.yaw) * localError.y) /
                 localError.yaw;
    } else {
      return 0.0;
    }
  }();
  return {outLinearVel, outRotationVel};
}