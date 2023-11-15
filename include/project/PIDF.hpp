struct pidf {
  double proportionalGain, integralGain, derivativeGain, feedForwardGain;
};
class pidfAlgorithm {
  pidf gains;
  double error = 0;
  double errorSum = 0;
  double prevError = 0;
  double setPoint = 0;

 public:
  pidfAlgorithm(pidf gains) : gains(gains) {}
  void setTarget(double itarget) { setPoint = itarget; }
  [[nodiscard]] double getOutput() const {
    return error * gains.proportionalGain + errorSum * gains.integralGain +
           gains.derivativeGain * (prevError - error) +
           setPoint * gains.feedForwardGain;
  };
  pidf getGains() { return gains; }
  void setGains(pidf igains) { gains = igains; };
  double poll(double reading) {
    error = setPoint - reading;
    return getOutput();
  }
};