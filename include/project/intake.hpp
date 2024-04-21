#pragma once
#include "main.h"
#include "project/Camera.hpp"
#include "project/catapult.hpp"
#include "project/geometry.hpp"
#include "settledUtil.hpp"

using hueRange = std::pair<int, int>;
// test
class Intake {
 public:
  enum class Mode;
  enum class State;
  Intake(pros::Motor motor, std::shared_ptr<Camera> camera);
  ~Intake();
  void setManualMode(bool manual, int32_t voltage = 12000);
  void waitUntilSettled(uint32_t timeoutMillis = TIMEOUT_MAX);
  bool isSettled();
  bool hasBall();
  bool seeBall();
  void release();

 private:
  void taskFunction();
  std::atomic_bool manual = false;
  pros::Motor motor;
  std::shared_ptr<Camera> camera;
  pros::Task internalTask;
  geometry::rectangle see_box = {-VISION_FOV_WIDTH / 2, 0, VISION_FOV_WIDTH,
                                 VISION_FOV_HEIGHT / 2};
  geometry::rectangle have_box = {-VISION_FOV_WIDTH / 2, 13, VISION_FOV_WIDTH,
                                  40};
  //   geometry::rectangle yellow_tag_box = {{0, 100}, {200, 140}};
  int32_t green_slot = 1;
  int32_t red_slot = 2;
  int32_t blue_slot = 3;
  pros::vision_signature green_sig =
      pros::c::vision_signature_from_utility(green_slot,
                                             -7391,
                                             -6289,
                                             -6840,
                                             -4551,
                                             -3071,
                                             -3811,
                                             6.000,
                                             0);
  pros::vision_signature red_sig =  //
      pros::c::vision_signature_from_utility(red_slot,
                                             6071,
                                             7771,
                                             6921,
                                             -747,
                                             -137,
                                             -442,
                                             3.500,
                                             0);
  pros::vision_signature blue_sig =
      pros::c::vision_signature_from_utility(blue_slot,
                                             -4681,
                                             -4083,
                                             -4382,
                                             10607,
                                             12549,
                                             11578,
                                             6.000,
                                             0);
};