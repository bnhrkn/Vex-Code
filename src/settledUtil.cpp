#include "project/settledUtil.hpp"
// constexpr SettledUtil::SettledUtil(double maxError, double maxErrorChange,
//                                    std::uint32_t minTime)
//     : maxError(maxError), maxErrorChange(maxErrorChange), minTime(minTime){};

void SettledUtil::step(double value, double target){
    if(std::abs(target - value) > maxError || std::abs(value - prevValue) > maxErrorChange){
        settled = false;
        prevTime = pros::millis();
    }
    else if(prevTime >= minTime){
        settled = true;
    }
    prevValue = value;
}
bool SettledUtil::isSettled() const{
    return settled;
}