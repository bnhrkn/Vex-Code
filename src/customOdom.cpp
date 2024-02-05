#include <utility>

#include "project/algorithms.hpp"
#include "project/customOdom.hpp"
using ScopeLock = std::scoped_lock<pros::Mutex>;
using namespace okapi::literals;

CustomOdom::CustomOdom(std::shared_ptr<okapi::ContinuousRotarySensor> left,
                       std::shared_ptr<okapi::ContinuousRotarySensor> right,
                       std::shared_ptr<pros::IMU> imu,
                       okapi::ChassisScales odomScales)
    : left(std::move(left)),
      right(std::move(right)),
      imu(std::move(imu)),
      scales(odomScales){};

CustomOdom::SensorValues CustomOdom::SensorValues::operator-(
    SensorValues values) const {
  return {x - values.x, angle - values.angle};
}

struct UnitlessPoint {
  double x = 0, y = 0;
  UnitlessPoint operator-(const UnitlessPoint& other) const {
    return {.x = x - other.x, .y = y - other.y};
  }
  UnitlessPoint operator+(const UnitlessPoint& other) const {
    return {.x = x + other.x, .y = y + other.y};
  }
  UnitlessPoint operator-() const { return {.x = -x, .y = -y}; }
  [[nodiscard]] UnitlessPoint rotate(const okapi::QAngle& angle) const {
    return rotate(angle.convert(okapi::radian));
  }
  [[nodiscard]] UnitlessPoint rotate(const double angleRad) const {
    return {.x = x * cos(angleRad) - y * sin(angleRad),
            .y = x * sin(angleRad) + y * cos(angleRad)};
  }
  [[nodiscard]] UnitlessPoint rotateAround(const UnitlessPoint& center,
                                           const double angleRad) const {
    return (*this - center).rotate(angleRad) + center;
  }
  [[nodiscard]] UnitlessPoint rotateAround(const UnitlessPoint& center,
                                           okapi::QAngle angle) const {
    return rotateAround(center, angle.convert(1_rad));
  }
};

// void CustomOdom::setScales(const okapi::ChassisScales &ichassisScales) {
//   ScopeLock lock(scalesMutex);
//   scales = ichassisScales;
// };

void CustomOdom::step() {
  using namespace okapi::literals;
  // ScopeLock stateLock(stateMutex);

  const auto reading =
      // Must update setState to match
      CustomOdom::SensorValues{
          (left->get() + right->get()) / 2.0 * okapi::degree,
          imu->get_rotation() * -1 * okapi::degree};

  // std::cout << std::format(
  //     "Left: {}, Right: {}, X: {}, Angle: {}\n", left->get(), right->get(),
  //     reading.x.convert(okapi::degree),
  //     reading.angle.convert(okapi::degree));

  const auto readingChange = reading - prevReading;
  // std::cout << std::format("DX: {}, DTheta: {}\n",
  //                          readingChange.x.convert(1_deg),
  //                          readingChange.angle.convert(1_deg));
  const auto prevThetaRad = prevReading.angle.convert(1_rad);
  const auto dThetaRad = readingChange.angle.convert(okapi::radian);

  constexpr auto degreesPerInch =  // Drivetrain specific constant
      360_deg * (48.0 / 36) / (2.75_in * std::numbers::pi);
  const auto arcLengthM = (readingChange.x / degreesPerInch).convert(1_m);

  // Where we were
  const UnitlessPoint prevPoint = {state.x.convert(1_m), state.y.convert(1_m)};

  const double avgAngleRad = prevThetaRad + dThetaRad / 2;

  // Change in theta must be nonzero beyond this point
  if (dThetaRad == 0) {  // Straight motion
    const UnitlessPoint localOffset = {arcLengthM, 0};
    const UnitlessPoint globalOffset = localOffset.rotate(avgAngleRad);
    const UnitlessPoint newPoint = prevPoint + globalOffset;
    state = okapi::OdomState{newPoint.x * 1_m, newPoint.y * 1_m, reading.angle};
  } else {
    // If dTheta is negative, the radius is "negative", but the point will be to
    // the right
    const auto arcRadiusM =
        arcLengthM / dThetaRad;  // May need to add offset. We assume that
                                 // distance rep. the center of the robot

    // Relative to the prevPoint, the center of the arc
    // const UnitlessPoint centerOffset{-sin(prevThetaRad) * arcRadiusM,
    //                                  cos(prevThetaRad) * arcRadiusM};

    // Chord length formula
    // chord = 2 * radius * sin (angle / 2)
    // can add another chord for the left-right translation if req'd
    const double chordLengthM = 2 * arcRadiusM * sin(dThetaRad / 2);

    const UnitlessPoint localOffset = {chordLengthM, 0};
    const UnitlessPoint globalOffset =
        localOffset.rotate(avgAngleRad);  // may need to be +
    const UnitlessPoint newPoint = prevPoint + globalOffset;

    // Absolute (field centric) center of the arc
    // const UnitlessPoint center = prevPoint + centerOffset;

    // Translate prevPoint by center (center is now origin), rotate, and
    // translate back Effect is that prevpoint rotates around the arbitrary
    // point center
    // const UnitlessPoint newPoint = prevPoint.rotateAround(center, dThetaRad);
    // const UnitlessPoint processPoint = prevPoint - center;
    // const UnitlessPoint rotatedProcessPoint = processPoint.rotate(dThetaRad);
    // const UnitlessPoint newPoint = rotatedProcessPoint + center;

    state = okapi::OdomState{newPoint.x * 1_m, newPoint.y * 1_m, reading.angle};
  }
  // auto xLocalChange =
  //     dThetaRad == 0 ? arcLengthM : 2 * sin(dThetaRad / 2) * arcRadiusM;
  // auto yLocalChange = 0;

  // auto xGlobalChang

  // ScopeLock stateLock(stateMutex);
  // state = {state.x + xChange, state.y + yChange, reading.angle};

  // const auto sinTheta = std::sin(dThetaRad);
  // const auto cosTheta = std::cos(dThetaRad);

  // double s, c;
  // // sin(x)/x and (1-cosx)/x with holes at x=0 corrected
  // if (std::abs(dThetaRad) < 1E-9) {
  //   s = 1.0 - 1.0 / 6.0 * dThetaRad * dThetaRad;
  //   c = 0.5 * dThetaRad;
  // } else {
  //   s = sinTheta / dThetaRad;
  //   c = (1 - cosTheta) / dThetaRad;
  // }

  // auto translation = okapi::Point{arcLengthM * s, arcLengthM * c};
  // auto rotatedTranslation = rotateAroundOrigin(translation, reading.angle);

  // state = {state.x + rotatedTranslation.x, state.y + rotatedTranslation.y,
  //          reading.angle};

  prevReading = reading;
}
okapi::OdomState CustomOdom::getState() const {
  // ScopeLock lock(stateMutex);
  return state;
};
okapi::Point CustomOdom::getPoint() const {
  const auto state = getState();
  // std::cout << std::format("Sending Point ({},{}) to getPoint caller\n",
  //                          state.x.convert(1_in), state.y.convert(1_in));
  return {state.x, state.y};
}
void CustomOdom::setState(const okapi::OdomState& istate) {
  // copeLock lock(stateMutex);
  state = istate;
  imu->set_rotation(-state.theta.convert(1_deg));
  left->reset();  // Very important?
  right->reset();
  prevReading = CustomOdom::SensorValues{
      (left->get() + right->get()) / 2.0 / 100 * okapi::degree,
      imu->get_rotation() * -1 * okapi::degree};
};