#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include "main.h"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/odometry/point.hpp"

namespace geometry {

struct rectangle {
  int32_t left_coord;
  int32_t top_coord;
  int32_t width;
  int32_t height;
  constexpr explicit rectangle(const pros::vision_object& object)
      : left_coord(object.left_coord),
        top_coord(object.top_coord),
        width(object.width),
        height(object.height) {}
  constexpr rectangle(int32_t left, int32_t top, int32_t width, int32_t height)
      : left_coord(left), top_coord(top), width(width), height(height) {}

  [[nodiscard]] constexpr int32_t right_coord() const {
    return left_coord + width;
  }
  [[nodiscard]] constexpr int32_t bottom_coord() const {
    return top_coord - height;
  }
  [[nodiscard]] constexpr int32_t center_x_coord() const {
    return left_coord + width / 2;
  }
  [[nodiscard]] constexpr int32_t center_y_coord() const {
    return top_coord - height / 2;
  }
  [[nodiscard]] constexpr bool operator==(const rectangle& other) const {
    return left_coord == other.left_coord && top_coord == other.top_coord &&
           width == other.width && height == other.height;
  }
  [[nodiscard]] constexpr int32_t area() const { return width * height; }
};

[[nodiscard]] constexpr std::optional<rectangle> intersection(
    const rectangle& a,
    const rectangle& b) {
  using std::min, std::max;
  int32_t left = std::max(a.left_coord, b.left_coord);
  int32_t right = std::min(a.right_coord(), b.right_coord());
  int32_t bottom = std::max(a.bottom_coord(), b.bottom_coord());
  int32_t top = std::min(a.top_coord, b.top_coord);

  if (left < right && bottom < top) {
    return rectangle{left, top, right - left, top - bottom};
  }
  return std::nullopt;
}

[[nodiscard]] constexpr bool intersects(const rectangle& a,
                                        const rectangle& b) {
  using std::min, std::max;
  return min(a.right_coord(), b.right_coord()) >
             max(a.left_coord, b.left_coord) &&
         min(a.top_coord, b.top_coord) >
             max(a.bottom_coord(), b.bottom_coord());
};

[[nodiscard]] auto rotateAroundOrigin(const okapi::OdomState& frame,
                                      const okapi::QAngle& angle)
    -> okapi::OdomState;
[[nodiscard]] auto rotateAroundOrigin(const okapi::Point& point,
                                      const okapi::QAngle& angle)
    -> okapi::Point;
[[nodiscard]] auto translatePoint(const okapi::OdomState& frame,
                                  const okapi::OdomState& delta)
    -> okapi::OdomState;
[[nodiscard]] auto translatePoint(const okapi::Point& point,
                                  const okapi::Point& delta) -> okapi::Point;

[[nodiscard]] auto angleToPoint(const okapi::Point& destination,
                                const okapi::Point& origin) -> okapi::QAngle;

[[nodiscard]] auto distanceToPoint(const okapi::Point& destination,
                                   const okapi::Point& origin)
    -> okapi::QLength;

// Returns the shortest angular distance from initial to final
[[nodiscard]] okapi::QAngle angleDifference(okapi::QAngle initial,
                                            okapi::QAngle final);
[[nodiscard]] okapi::QAngle angleDistance(okapi::QAngle a, okapi::QAngle b);

[[nodiscard]] okapi::QAngle normalize360(okapi::QAngle angle);
[[nodiscard]] okapi::QAngle normalize180(okapi::QAngle angle);
}  // namespace geometry
