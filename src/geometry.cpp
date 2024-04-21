#include "project/geometry.hpp"
#include <algorithm>

namespace geometry {

using namespace okapi::literals;

static_assert(intersects({-106, -31, 126, 74}, {-158, 106, 316, 212}));
// Rotates a vector around the origin by a certain angle
auto rotateAroundOrigin(const okapi::OdomState& frame,
                        const okapi::QAngle& angle) -> okapi::OdomState {
  const auto& [x, y, yaw] = frame;

  const auto& radAngle = angle.convert(okapi::radian);
  return {x * std::cos(radAngle) - y * std::sin(radAngle),
          x * std::sin(radAngle) + y * std::cos(radAngle), yaw - angle};
}

auto rotateAroundOrigin(const okapi::Point& point, const okapi::QAngle& angle)
    -> okapi::Point {
  const auto& [x, y] = point;
  const auto& radAngle = angle.convert(okapi::radian);
  return {x * std::cos(radAngle) - y * std::sin(radAngle),
          x * std::sin(radAngle) + y * std::cos(radAngle)};
}

// Translate a vector by a certain delta. Theta in delta has no effect.
auto translatePoint(const okapi::OdomState& frame,
                    const okapi::OdomState& delta) -> okapi::OdomState {
  return {frame.x - delta.x, frame.y - delta.y, frame.theta};
}
auto translatePoint(const okapi::Point& point, const okapi::Point& delta)
    -> okapi::Point {
  return {point.x - delta.x, point.y - delta.y};
}

auto angleToPoint(const okapi::Point& destination, const okapi::Point& origin)
    -> okapi::QAngle {
  auto point = translatePoint(destination, origin);
  return std::atan2(point.y.convert(1_m), point.x.convert(1_m)) * okapi::radian;
}

auto distanceToPoint(const okapi::Point& destination,
                     const okapi::Point& origin) -> okapi::QLength {
  auto point = translatePoint(destination, origin);
  return std::hypot(point.x.convert(1_m), point.y.convert(1_m)) * 1_m;
}

[[nodiscard]] okapi::QAngle angleDifference(okapi::QAngle initial,
                                            okapi::QAngle final) {
  return normalize180(final - initial);
}

[[nodiscard]] okapi::QAngle angleDistance(okapi::QAngle a, okapi::QAngle b) {
  return std::abs(normalize180(a - b).convert(okapi::radian)) * okapi::radian;
}

[[nodiscard]] okapi::QAngle normalize360(okapi::QAngle angle) {
  angle = fmod(angle.convert(okapi::degree), 360) * okapi::degree;
  if (angle < 0_deg) {
    angle += 360_deg;
  }
  return angle;
}

[[nodiscard]] okapi::QAngle normalize180(okapi::QAngle angle) {
  angle = fmod(angle.convert(okapi::degree) + 180, 180) * okapi::degree;
  if (angle < 0_deg) {
    angle += 360_deg;
  }
  return angle - 180_deg;
}

}  // namespace geometry