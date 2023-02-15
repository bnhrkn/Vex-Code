#pragma once
#include "main.h"

class Indexer {
    public:
    Indexer(pros::Distance idistance, std::uint32_t thresholdTime);
    ~Indexer();
    int getCount() const;
    protected:
    pros::Distance distance;
    std::atomic<int> discCount = 0;
    const int threshold;
    pros::Task task;
    void taskFunction();

};