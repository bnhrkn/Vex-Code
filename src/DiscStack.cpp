#include "project/DiscStack.hpp"
#include "project/algorithms.hpp"

auto getReading(pros::Distance &distance) -> int {
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

Indexer::Indexer(pros::Distance idistance, std::uint32_t thresholdTime)
    : distance(idistance), threshold(thresholdTime),
      task([&]() { taskFunction(); }){};
Indexer::~Indexer() { task.remove(); }

void Indexer::taskFunction() {
  auto debounce = debouncer(int{0}, threshold, getReading(distance));
  while (!pros::Task::notify_take(true, 10)) {
    debounce.poll(getReading(distance));
    if (debounce.get() != discCount.load())
      discCount.store(debounce.get());
  }
}

int Indexer::getCount() const { return discCount.load(); }
