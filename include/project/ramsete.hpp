#pragma once
#include "geometry/pose.hpp"
#include "geometry/profilepoint.hpp"
#include "main.h"

class RamseteController {
public:
  RamseteController(const double ib, const double izeta);
  std::pair<double, double> step(const squiggles::Pose&, const squiggles::ProfilePoint&);

protected:
  double b, zeta, gain;
};