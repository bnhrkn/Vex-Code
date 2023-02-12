#pragma once
#include "main.h"

class Indexer {
    public:
    Indexer(pros::Distance idistance, pros::ADIDigitalOut icylinder, std::uint32_t thresholdTime);
    ~Indexer();
    std::uint32_t getCount();
    void decrementCount();
    protected:
    pros::Distance distance;
    pros::ADIDigitalOut cylinder;
    std::atomic<std::uint32_t> discCount = 0;
    std::uint32_t threshold;
    pros::Task task;
    void taskFunction();

};