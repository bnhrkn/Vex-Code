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

ChassisSpeeds ramsete(const squiggles::Pose& nowPose,
                      const squiggles::ProfilePoint& goalPoint,
                      double b,
                      double zeta);

[[nodiscard]] auto stateToPose(const okapi::OdomState state) -> squiggles::Pose;
[[nodiscard]] auto stateToPoint(const okapi::OdomState state) -> okapi::Point;
[[nodiscard]] auto chassisToTankSpeeds(
    const ChassisSpeeds speeds,
    const okapi::ChassisScales scales,
    const okapi::AbstractMotor::GearsetRatioPair) -> TankSpeeds;

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

[[nodiscard]] auto distanceCalcRPM(const okapi::QLength& distance)
    -> okapi::QAngularSpeed;

template <typename T>
  requires std::is_signed_v<T>
[[nodiscard]] auto mapIntoRange(const double value,
                                const std::pair<T, T> fromRange,
                                const std::pair<T, T> toRange) {
  return (toRange.second - toRange.first) * (value - fromRange.first) /
             (fromRange.second - fromRange.min) +
         toRange.first;
}

template <typename T>
  requires std::three_way_comparable<T> && std::is_signed_v<T>
class debouncer {
 public:
  debouncer(T changeThreshold, std::uint32_t timeThreshold, T initValue)
      : changeThreshold(changeThreshold),
        timeThreshold(timeThreshold),
        process(initValue),
        output(process) {}

  [[nodiscard]] T get() const { return output; }
  [[nodiscard]] bool isStable() const { return stable; };

  void poll(T value) {
    if (std::abs(value - process) > changeThreshold) {
      stable = false;
      process = value;
      time = pros::millis();
    } else if (pros::millis() - time > timeThreshold && process != output) {
      output = process;
      stable = true;
    }
  }

 protected:
  bool stable = false;
  const T changeThreshold;
  const std::uint32_t timeThreshold;
  T process;
  T output;
  std::uint32_t time = 0;
};

template <typename T>
  requires std::three_way_comparable<T>
class changeLimiter {
 public:
  changeLimiter(T maxChangePerSec, T initValue = 0)
      : maxChangePerSec(maxChangePerSec),
        prevValue(initValue),
        prevTime(pros::millis()){};
  T filter(T value) {
    auto dt = pros::millis() - prevTime;
    auto change = value - prevValue;
    auto scaledChange = change * 1000 / dt;
    prevValue = value;
    prevTime = pros::millis();
    return prevValue + std::clamp(value, -maxChangePerSec, maxChangePerSec);
  };
  T filterIncrease(T value) {
    auto dt = pros::millis() - prevTime;
    auto change = value - prevValue;
    auto scaledMaxChange = maxChangePerSec * dt / 1000.0;
    prevValue = value;
    prevTime = pros::millis();
    return prevValue + (change >= scaledMaxChange ? scaledMaxChange : change);
  };
  // [[nodiscard]] T getOutput() const;

 protected:
  const T maxChangePerSec;
  T prevValue = 0;
  std::uint32_t prevTime;
};

auto inRange(auto num, std::pair<decltype(num), decltype(num)> range) -> bool {
  auto minMax = std::minmax(range.first, range.second);
  return (num > std::get<0>(minMax)) && (num < std::get<1>(minMax));
};

class lowPassFilter : public okapi::Filter {
 public:
  lowPassFilter(double cutoffFreq, double deltaTime);
  double filter(double value);
  [[nodiscard]] double getOutput() const;

 protected:
  double output = 0;
  const double ePow;
};

class DisconnectDetector {
 public:
  bool changedToConnected();

 private:
  static bool connected();
  bool prevConnected = connected();
};

double sinc(double radians);