#include "project/tracker.hpp"
#include "project/algorithms.hpp"
#include "project/geometry.hpp"

Tracker::Tracker(std::unique_ptr<Odometry> odometry,
                 std::unique_ptr<pros::GPS> gps,
                 AllianceColor color)
    : color(color), odometry(std::move(odometry)), gps(std::move(gps)) {
  this->step();
};

[[nodiscard]] okapi::OdomState Tracker::getState() const {
  return this->state;
}

[[nodiscard]] okapi::Point Tracker::getPoint() const {
  return {this->state.x, this->state.y};
}

void Tracker::setState(const okapi::OdomState& state) {
  odometry->setState(state);
  this->state = state;
}

void Tracker::setColor(AllianceColor color) {
  this->color = color;
};

[[nodiscard]] AllianceColor Tracker::getColor() const {
  return color;
};

[[nodiscard]] okapi::OdomState Tracker::getGPSState() const {
  using namespace okapi::literals;
  pros::gps_status_s_t status = gps->get_status();
  okapi::OdomState reading{status.x * okapi::meter, status.y * okapi::meter,
                           status.yaw * okapi::degree};

  reading.theta = geometry::normalize360(reading.theta + 90_deg);
  if (color == AllianceColor::red) {
    // Flip the coordinates so gps agrees with odom
    reading = {-reading.x, -reading.y,
               geometry::normalize360(reading.theta + 180_deg)};
  }

  return reading;
}
void Tracker::step() {
  odometry->step();
  state = odometry->getState();
  // if (gps->get_error() * okapi::meter > 2 * okapi::inch) {
  //   odometry->step();
  //   state = odometry->getState();
  // } else {
  //   std::cout << std::format("Tared with est error {}\n", gps->get_error());
  //   state = getGPSState();
  //   odometry->setState(state);
  // }
}
