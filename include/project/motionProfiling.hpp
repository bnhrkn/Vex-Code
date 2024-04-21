#pragma once
#include "main.h"

struct ProfilePoint {
  okapi::OdomState pose = {0 * okapi::meter, 0 * okapi::meter,
                           0 * okapi::radian};
  okapi::QSpeed linearVel = 0 * okapi::mps;
  okapi::QAcceleration linearAccel = 0 * okapi::mps2;
  okapi::QAngularSpeed angularVel = 0 * okapi::radps;
  okapi::QAngularAcceleration angularAccel = 0 * okapi::radps / okapi::second;
  okapi::QTime time = 0 * okapi::second;
};

struct Constraints {
  okapi::QSpeed maxVel = 0 * okapi::mps;
  okapi::QAcceleration maxAccel = 0 * okapi::mps2;
  okapi::QAngularSpeed maxAngularVel = 0 * okapi::radps;
  okapi::QAngularAcceleration maxAngularAccel =
      0 * okapi::radps / okapi::second;
};

[[nodiscard]] std::vector<ProfilePoint> genLinearProfile(
    okapi::Point start,
    okapi::Point end,
    Constraints constraints,
    okapi::QTime dt,
    bool reverse = false);

[[nodiscard]] std::vector<ProfilePoint> genAngularProfile(
    okapi::OdomState start,
    okapi::QAngle end,
    Constraints constraints,
    okapi::QTime dt
    // bool reverse = false
);

// [[nodiscard]] std::vector<ProfilePoint> genCurvyProfile(
//     std::vector<okapi::OdomState> path,
//     Constraints constraints,
//     okapi::QTime dt,
//     bool reverse = false);

[[nodiscard]] std::vector<ProfilePoint> filterSquigglesProfile(
    const std::vector<squiggles::ProfilePoint>& path);

// class MotionProfiler {
//  public:
//   enum class ProfileType {
//     LINEAR = 0,
//     ANGULAR = 1,
//     CURVY = 2,
//   };
//   explicit MotionProfiler(Constraints constraints);
//   std::vector<ProfilePoint>
// };
