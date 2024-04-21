#pragma once
#include "geometry.hpp"
#include "main.h"

class Camera {
 public:
  explicit Camera(pros::Vision&& vision);
  enum class Color {
    RED = 1,
    GREEN = 2,
    BLUE = 3,
  };
  // All geometric objects are returned in a standard cartesian coordinate
  // plane. 0,0 is the center of the screen. Positive x is right, positive y is
  // up. The maximum x magnitude is VISION_FOV_WIDTH / 2 and the maximum y
  // magnitude is VISION_FOV_HEIGHT /2
  std::vector<geometry::rectangle> read_objects(
      std::initializer_list<Color> colors,
      std::size_t max_objects = 64);
  void set_signature(Color color, pros::vision_signature_s_t signature);
  pros::vision_signature_s_t get_signature(Color color);
  void set_exposure(int32_t exposure);
  int32_t get_exposure();
  void set_white_balance(bool enable_auto = true);
  void set_white_balance(int32_t white_balance);
  int32_t get_white_balance();

 private:
  pros::Vision vision;
  std::map<Color, pros::vision_signature_s_t> signatures;
  int32_t exposure = 0;
  int32_t white_balance = 0;
};

constexpr geometry::rectangle vision_default_to_cartesian(
    geometry::rectangle rect) {
  return {rect.left_coord - VISION_FOV_WIDTH / 2,
          -(rect.top_coord - VISION_FOV_HEIGHT / 2), rect.width, rect.height};
}
static_assert(vision_default_to_cartesian({VISION_FOV_WIDTH / 2,
                                           VISION_FOV_HEIGHT / 2, 10, 10}) ==
              geometry::rectangle{0, 0, 10, 10});

static_assert(vision_default_to_cartesian({0, 0, 15, 15}) ==
              geometry::rectangle{-VISION_FOV_WIDTH / 2, VISION_FOV_HEIGHT / 2,
                                  15, 15});