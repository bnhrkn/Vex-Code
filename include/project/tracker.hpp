#pragma once
#include "project/Odometry.hpp"
#include "pros/gps.hpp"

enum class AllianceColor { red, blue };

class Tracker {
 public:
  Tracker(std::unique_ptr<Odometry> odometry,
          std::unique_ptr<pros::GPS> gps,
          AllianceColor color = AllianceColor::red);
  void step();
  [[nodiscard]] okapi::OdomState getState() const;
  [[nodiscard]] okapi::Point getPoint() const;
  void setState(const okapi::OdomState& istate);
  void setColor(AllianceColor color);
  [[nodiscard]] AllianceColor getColor() const;

 private:
  [[nodiscard]] okapi::OdomState getGPSState() const;
  AllianceColor color;
  std::unique_ptr<Odometry> odometry;
  std::unique_ptr<pros::GPS> gps;
  okapi::OdomState state{0 * okapi::inch, 0 * okapi::inch, 0 * okapi::degree};
};
