#include "project/motionProfiling.hpp"
#include <cmath>
#include "project/geometry.hpp"

using QUnitless = okapi::
    RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>>;

[[nodiscard]] std::vector<ProfilePoint> genLinearProfile(
    okapi::Point start,
    okapi::Point end,
    Constraints constraints,
    okapi::QTime dt,
    bool reverse) {
  using namespace okapi::literals;
  const okapi::QLength distance = std::hypot((end.x - start.x).convert(1_m),
                                             (end.y - start.y).convert(1_m)) *
                                  1_m;
  const okapi::QLength distance_x = end.x - start.x;
  const okapi::QLength distance_y = end.y - start.y;
  const okapi::QAngle heading =
      std::atan2(distance_y.convert(1_m), distance_x.convert(1_m)) * 1_rad +
      static_cast<int>(reverse) * 180_deg;
  const okapi::QAcceleration& maxAccel = constraints.maxAccel;
  const okapi::QSpeed& maxVel = constraints.maxVel;
  const int32_t reverse_coeff = static_cast<int>(reverse) * -2 + 1;

  const okapi::QSpeed topSpeed = std::min(sqrt(maxAccel * distance), maxVel);
  const okapi::QTime accelDuration = topSpeed / maxAccel;
  const okapi::QTime coastDuration =
      std::max(distance / topSpeed - accelDuration, 0_s);
  const okapi::QTime decelDuration = accelDuration;
  const okapi::QTime accelEnd = accelDuration;
  const okapi::QTime coastEnd = accelEnd + coastDuration;
  const okapi::QTime decelEnd = coastEnd + decelDuration;

  std::cout << std::format(
      "distance {}, distance_x {}, distance_y {}, heading {}, maxAccel {},"
      "maxVel {}, reverse_coeff{}, topSpeed {}, accelDuration {}, "
      "coastDuration {}, "
      "decelDuration {}, accelEnd {}, coastEnd {}, "
      "decelEnd {}\n",
      distance.convert(1_m), distance_x.convert(1_m), distance_y.convert(1_m),
      heading.convert(1_rad), maxAccel.convert(1_mps2), maxVel.convert(1_mps),
      reverse_coeff, topSpeed.convert(1_mps), accelDuration.convert(1_s),
      coastDuration.convert(1_s), decelDuration.convert(1_s),
      accelEnd.convert(1_s), coastEnd.convert(1_s), decelEnd.convert(1_s));

  std::vector<ProfilePoint> path;
  path.reserve(
      static_cast<size_t>(std::ceil((decelEnd / dt).convert(QUnitless(1.0)))));

  for (okapi::QTime t = 0_s; t < accelEnd; t += dt) {
    const okapi::QLength position = 0.5 * maxAccel * t * t;
    const okapi::QSpeed velocity = maxAccel * t;
    const QUnitless position_scaler = position / distance;
    const okapi::QLength x_coord = position_scaler * distance_x;
    const okapi::QLength y_coord = position_scaler * distance_y;
    ProfilePoint point = {
        {x_coord, y_coord, heading},      velocity * reverse_coeff,
        maxAccel * reverse_coeff,         0 * okapi::radps,
        0 * okapi::radps / okapi::second, t};
    path.push_back(point);
  };

  const okapi::QLength accelDist =
      0.5 * maxAccel * accelDuration * accelDuration;

  for (okapi::QTime t = accelEnd; t < coastEnd; t += dt) {
    const okapi::QLength position = accelDist + topSpeed * (t - accelEnd);
    const okapi::QSpeed velocity = topSpeed;
    const QUnitless position_scaler = position / distance;
    const okapi::QLength x_coord = position_scaler * distance_x;
    const okapi::QLength y_coord = position_scaler * distance_y;
    ProfilePoint point = {{x_coord, y_coord, heading},
                          velocity * reverse_coeff,
                          0_mps2,
                          0 * okapi::radps,
                          0 * okapi::radps / 1_s,
                          t};
    path.push_back(point);
  }
  const okapi::QLength coastDist = topSpeed * coastDuration;

  for (okapi::QTime t = coastEnd; t <= decelEnd; t += dt) {
    const okapi::QLength position =
        accelDist + coastDist + topSpeed * (t - coastEnd) -
        0.5 * maxAccel * (t - coastEnd) * (t - coastEnd);
    const okapi::QSpeed velocity = topSpeed - maxAccel * (t - coastEnd);
    const QUnitless position_scaler = position / distance;
    const okapi::QLength x_coord = position_scaler * distance_x;
    const okapi::QLength y_coord = position_scaler * distance_y;
    ProfilePoint point = {
        {x_coord, y_coord, heading},      velocity * reverse_coeff,
        -1 * maxAccel * reverse_coeff,    0 * okapi::radps,
        0 * okapi::radps / okapi::second, t};
    path.push_back(point);
  }
  return path;
};

[[nodiscard]] std::vector<ProfilePoint> genAngularProfile(
    okapi::OdomState start,
    okapi::QAngle end,
    Constraints constraints,
    okapi::QTime dt
    // bool reverse
) {
  using namespace okapi::literals;

  const okapi::QAngle displacement = geometry::angleDistance(end, start.theta);
  const int32_t reverse_coeff = displacement >= 0_rad ? 1 : -1;

  const okapi::QAngularAcceleration maxAccel =
      constraints.maxAngularAccel * reverse_coeff;
  const okapi::QAngularSpeed maxVel = constraints.maxAngularVel * reverse_coeff;
  const okapi::QAngularSpeed topVel =
      std::min(sqrt(abs(maxAccel * displacement)), abs(maxVel)) * reverse_coeff;
  const okapi::QTime accelDuration = topVel / maxAccel;
  const okapi::QTime coastDuration =
      std::max(displacement / topVel - accelDuration, 0_s);
  const okapi::QTime decelDuration = accelDuration;
  const okapi::QTime accelEnd = accelDuration;
  const okapi::QTime coastEnd = accelEnd + coastDuration;
  const okapi::QTime decelEnd = coastEnd + decelDuration;

  //   std::cout << std::format(
  //       "distance {}, distance_x {}, distance_y {}, heading {}, maxAccel {},"
  //       "maxVel {}, reverse_coeff{}, topSpeed {}, accelDuration {}, "
  //       "coastDuration {}, "
  //       "decelDuration {}, accelEnd {}, coastEnd {}, "
  //       "decelEnd {}\n",
  //       distance.convert(1_m), distance_x.convert(1_m),
  //       distance_y.convert(1_m), heading.convert(1_rad),
  //       maxAccel.convert(1_mps2), maxVel.convert(1_mps), reverse_coeff,
  //       topSpeed.convert(1_mps), accelDuration.convert(1_s),
  //       coastDuration.convert(1_s), decelDuration.convert(1_s),
  //       accelEnd.convert(1_s), coastEnd.convert(1_s), decelEnd.convert(1_s));

  std::vector<ProfilePoint> path;

  path.reserve(
      static_cast<size_t>(std::ceil((decelEnd / dt).convert(QUnitless(1.0)))));
  for (okapi::QTime t = 0_s; t < accelEnd; t += dt) {
    const okapi::QAngle heading = 0.5 * maxAccel * t * t;
    const okapi::QAngularSpeed velocity = maxAccel * t;
    ProfilePoint point = {{start.x, start.y, heading + start.theta},
                          0_mps,
                          0_mps2,
                          velocity,
                          maxAccel,
                          t};
    path.push_back(point);
  };

  const okapi::QAngle accelDisp =
      0.5 * maxAccel * accelDuration * accelDuration;

  for (okapi::QTime t = accelEnd; t < coastEnd; t += dt) {
    const okapi::QAngle heading = accelDisp + topVel * (t - accelEnd);
    const okapi::QAngularSpeed velocity = topVel;
    ProfilePoint point = {
        {start.x, start.y, heading + start.theta}, 0_mps, 0_mps2, velocity,
        0 * okapi::radps / okapi::second,          t};
    path.push_back(point);
  }

  const okapi::QAngle coastDisp = topVel * coastDuration;

  for (okapi::QTime t = coastEnd; t <= decelEnd; t += dt) {
    const okapi::QAngle heading =
        accelDisp + coastDisp + topVel * (t - coastEnd) -
        0.5 * maxAccel * (t - coastEnd) * (t - coastEnd);
    const okapi::QAngularSpeed velocity = topVel - maxAccel * (t - coastEnd);
    ProfilePoint point = {{start.x, start.y, heading + start.theta},
                          0_mps,
                          0_mps2,
                          velocity,
                          -1 * maxAccel,
                          t};
    path.push_back(point);
  }
  return path;
};

[[nodiscard]] std::vector<ProfilePoint> filterSquigglesProfile(
    const std::vector<squiggles::ProfilePoint>& path) {
  using namespace okapi::literals;
  std::vector<ProfilePoint> filteredPath;
  filteredPath.reserve(path.size());
  for (const auto& point : path) {
    filteredPath.emplace_back(
        okapi::OdomState{point.vector.pose.x * 1_m, point.vector.pose.y * 1_m,
                         point.vector.pose.yaw * 1_rad},
        point.vector.vel * 1_mps, point.vector.accel * 1_mps2,
        point.curvature * point.vector.vel * 1_rad / 1_s,
        point.curvature * point.vector.accel * 1_rad / 1_s / 1_s,
        point.time * 1_s);
  }
  return filteredPath;
}

// [[nodiscard]] std::vector<ProfilePoint> genCurvyProfile(
//     std::vector<okapi::OdomState> path,
//     Constraints constraints,
//     okapi::ChassisScales scales,
//     okapi::QTime dt,
//     bool reverse) {
//   using namespace okapi::literals;
//   squiggles::Constraints squigglesConstraints = {
//       constraints.maxVel.convert(1_mps),
//       constraints.maxAccel.convert(1_mps2),
//       std::numeric_limits<double>::max(),
//       constraints.maxAngularVel.convert(okapi::radps)};
//       squiggles::PhysicalModel model = {};
//   squiggles::SplineGenerator generator()
// }
