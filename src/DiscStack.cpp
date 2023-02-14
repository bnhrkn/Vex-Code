#include "project/DiscStack.hpp"

auto getReading(pros::Distance &distance) -> std::uint32_t {
  auto value = distance.get();
  if (value > 85)
    return 0;
  if (value > 70)
    return 1;
  if (value > 50)
    return 2;
  if (value > 25)
    return 3;
  return 4; // Disc is passing, cannot hold 4
}

Indexer::Indexer(pros::Distance idistance, pros::ADIDigitalOut icylinder,
                 std::uint32_t thresholdTime)
    : distance(idistance), cylinder(icylinder), threshold(thresholdTime),
      task([&]() { taskFunction(); }){
      };
Indexer::~Indexer() { task.notify(); }

void Indexer::taskFunction() {
  uint32_t time = 0;
  auto process = discCount.load();
  while (!pros::Task::notify_take(true, 10)) {
    auto reading = getReading(distance);
    //std::cout << "reading: " << reading << " process: " << process << std::endl;
    if (reading != process) {
      process = reading;
      time = pros::millis();
    } else if (pros::millis() - time > threshold && process != discCount.load()) {
      discCount.store(process);
    }
  }
}

std::uint32_t Indexer::getCount(){
    return discCount.load();
}
void Indexer::decrementCount(){ // Not working, is immediately undone in taskFunction
    discCount--;
}
