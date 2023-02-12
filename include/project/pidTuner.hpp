#pragma once
#include "main.h"

template <std::size_t size, auto getter, auto setter>
  requires std::invocable<decltype(setter), std::array<double, size>> &&
           std::same_as<decltype(std::invoke(getter)), std::array<double, size>>
class PIDConstantsTuner {
public:
  void inc() {
    auto gains = getter();
    gains[whichGain] += increment;
    setter(gains);
  };
  void dec() {
    auto gains = getter();
    gains[whichGain] -= increment;
    setter(gains);
  };
  void incPrecision() { increment *= 10.0; };
  void decPresicion() { increment /= 10.0; };
  void nextGain() {
    whichGain = whichGain + 1;
    whichGain = whichGain % size;
  };

private:
  std::size_t whichGain = 0;
  double increment = 0.001;
};