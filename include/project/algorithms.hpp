#pragma once
#include "geometry/pose.hpp"
#include "geometry/profilepoint.hpp"
#include "main.h"

// class RamseteController {
// public:
//   RamseteController(const double ib, const double izeta);
//   std::pair<double, double> step(const squiggles::Pose &,
//                                  const squiggles::ProfilePoint &);

// protected:
//   double b, zeta, gain;
// };
struct ChassisSpeeds {
  okapi::QSpeed linearVel;
  okapi::QAngularSpeed angularVel;
};
struct TankSpeeds {
  okapi::QAngularSpeed left, right;
};

ChassisSpeeds ramsete(const squiggles::Pose &nowPose,
                      const squiggles::ProfilePoint &goalPoint, double b,
                      double zeta);

auto stateToPose(okapi::OdomState state) -> squiggles::Pose;
auto chassisToTankSpeeds(const ChassisSpeeds speeds,
                         const okapi::ChassisScales scales,
                         const okapi::AbstractMotor::GearsetRatioPair)
    -> TankSpeeds;
auto getConvertedState(const std::shared_ptr<okapi::Odometry> &odometry)
    -> okapi::OdomState;
auto convertState(const okapi::OdomState &state) -> okapi::OdomState;

auto rotateAroundOrigin(const okapi::OdomState &frame,
                        const okapi::QAngle &angle) -> okapi::OdomState;
auto translatePoint(const okapi::OdomState &frame,
                    const okapi::OdomState &delta) -> okapi::OdomState;

template <typename T>
  requires std::three_way_comparable<T> && std::is_signed_v<T>
class debouncer {
public:
  debouncer(T changeThreshold, std::uint32_t timeThreshold, T initValue)
      : changeThreshold(changeThreshold), timeThreshold(timeThreshold),
        process(initValue), output(process) {}

  T get() { return output; }

  void poll(T value) {
    if (std::abs(value - process) > changeThreshold) {
      process = value;
      time = pros::millis();
    } else if (pros::millis() - time > timeThreshold && process != output) {
      output = process;
    }
  }

protected:
  const T changeThreshold;
  const std::uint32_t timeThreshold;
  T process;
  T output;
  std::uint32_t time = 0;
};

auto inRange(auto num, std::pair<decltype(num), decltype(num)> range) -> bool {
  auto minMax = std::minmax(range.first, range.second);
  return (num > std::get<0>(minMax)) && (num < std::get<1>(minMax));
};
