#pragma once
#include <concepts>
#include "geometry/pose.hpp"
#include "geometry/profilepoint.hpp"
#include "main.h"
#include "project/motionProfiling.hpp"

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
ChassisSpeeds ramsete(const okapi::OdomState& nowPose,
                      const ProfilePoint& goalPoint,
                      double b = 2.0,
                      double zeta = 0.7);

[[nodiscard]] auto stateToPose(okapi::OdomState state) -> squiggles::Pose;
[[nodiscard]] auto stateToPoint(okapi::OdomState state) -> okapi::Point;
[[nodiscard]] auto chassisToTankSpeeds(ChassisSpeeds speeds,
                                       okapi::ChassisScales scales,
                                       okapi::AbstractMotor::GearsetRatioPair)
    -> TankSpeeds;

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

 private:
  bool stable = false;
  T changeThreshold;
  std::uint32_t timeThreshold;
  T process;
  T output;
  std::uint32_t time = 0;
};

template <typename T>
  requires std::three_way_comparable<T>
class changeLimiter {
 public:
  explicit changeLimiter(T maxChangePerSec, T initValue = 0)
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

 private:
  T maxChangePerSec;
  T prevValue = 0;
  std::uint32_t prevTime;
};

auto inRange(auto num, std::pair<decltype(num), decltype(num)> range) -> bool {
  auto minMax = std::minmax(range.first, range.second);
  return (num > std::get<0>(minMax)) && (num < std::get<1>(minMax));
};
template <typename T>
concept bool_invocable = requires(T c) {
  { c() } -> std::convertible_to<bool>;
};

bool waitFor(bool_invocable auto&& func,
             std::uint32_t deadline = std::numeric_limits<uint32_t>::max(),
             std::uint32_t interval = 10) {
  auto startTime = pros::millis();
  while (pros::millis() < deadline) {
    if (std::forward<decltype(func)>(func)()) {
      return true;
    }
    pros::Task::delay_until(&startTime, interval);
  }
  return false;
}

class lowPassFilter : public okapi::Filter {
 public:
  lowPassFilter(double cutoffFreq, double deltaTime);
  double filter(double value) override;
  [[nodiscard]] double getOutput() const override;

 private:
  double output = 0;
  double ePow;
};

class DisconnectDetector {
 public:
  bool changedToConnected();

 private:
  static bool connected();
  bool prevConnected = connected();
};

double sinc(double radians);

// template <std::three_way_comparable T>
template <typename T>
constexpr bool approximatelyEqual(T a, T b, T epsilon) {
  T diff = a - b;
  if (diff < T(0.0)) {
    diff = -diff;
  }
  return diff < epsilon;
};