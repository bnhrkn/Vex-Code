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

auto rotateAroundOrigin(const okapi::OdomState &frame, const okapi::QAngle &angle) -> okapi::OdomState;
auto translatePoint(const okapi::OdomState &frame, const okapi::OdomState &delta) -> okapi::OdomState;


