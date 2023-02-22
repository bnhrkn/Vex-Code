#pragma once
#include "main.h"

class SettledUtil {
    public:
    constexpr SettledUtil(double maxError, double maxErrorChange,
                                   std::uint32_t minTime)
    : maxError(maxError), maxErrorChange(maxErrorChange), minTime(minTime){};
    // void reset(std::uint32_t time, double value, bool settled);
    void step(double value, double target);
    bool isSettled() const;
protected:
    std::uint32_t prevTime = 0;
    double prevValue = 0;
    bool settled = false;
    const double maxError;
    const double maxErrorChange;
    const uint32_t minTime;
};