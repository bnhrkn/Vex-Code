#include "project/Camera.hpp"
#include <algorithm>

Camera::Camera(pros::Vision&& vision) : vision(std::move(vision)) {
  vision.set_zero_point(VISION_ZERO_CENTER);
  vision.set_auto_white_balance(true);
}

std::vector<geometry::rectangle> Camera::read_objects(
    std::initializer_list<Color> colors,
    size_t max_objects) {
  std::vector<geometry::rectangle> rectangles;
  rectangles.reserve(max_objects);

  size_t num_objects = vision.get_object_count();
  // std::cout << "num_objects: " << num_objects << "\n";
  for (size_t i = 0; i < num_objects && i < max_objects; i++) {
    pros::vision_object object = vision.get_by_size(i);
    auto left_c = object.left_coord;
    auto top_c = object.top_coord;
    auto width_c = object.width;
    auto height_c = object.height;
    // std::cout << "object.signature: " << object.signature << "\n";

    if (object.signature == VISION_OBJECT_ERR_SIG) {
      std::cout << "Error reading object " << i << "\n";
      continue;  // Should never happen
    }
    for (auto color : colors) {
      if (object.signature == signatures[color].id) {
        // Changes +y to upwards as it should be by negative top_coord
        auto left = object.left_coord;
        auto top = object.top_coord;
        auto width = object.width;
        auto height = object.height;
        rectangles.emplace_back(left, top, width, height);
        break;
      }
    }
  }
  rectangles.shrink_to_fit();
  return rectangles;
}

void Camera::set_signature(Color color, pros::vision_signature_s_t signature) {
  signatures[color] = signature;
  vision.set_signature(static_cast<int>(color), &signature);
};
pros::vision_signature_s_t Camera::get_signature(Color color) {
  return signatures[color];
};
void Camera::set_exposure(int32_t exposure) {
  vision.set_exposure(exposure);
};
int32_t Camera::get_exposure() {
  return vision.get_exposure();
};
void Camera::set_white_balance(bool enable_auto) {
  vision.set_auto_white_balance(enable_auto);
};
void Camera::set_white_balance(int32_t white_balance) {
  vision.set_auto_white_balance(false);
  vision.set_white_balance(white_balance);
};
int32_t Camera::get_white_balance() {
  return vision.get_white_balance();
};