#pragma once
#include <algorithm>
#include <cmath>
#include <utility>
namespace remote5g {
inline double clamp(double v, double lo, double hi) { return std::clamp(v, lo, hi); }
inline double approach(double v, double target, double step) {
  return v + clamp(target-v, -step, step);
}
inline std::pair<double,double> ackermann(double angle, double length, double track) {
  const double t=std::tan(angle);
  return {std::atan2(t, 1-track*t/(2*length)), std::atan2(t, 1+track*t/(2*length))};
}
inline bool twist(double v, double omega, double length, double &angle) {
  if (!std::isfinite(v) || !std::isfinite(omega)) return false;
  if (std::abs(v)<1e-4) { angle=0; return std::abs(omega)<1e-4; }
  angle=std::atan(length*omega/v); return true;
}
inline double speed_step(double current, double target, double accel, double decel, double dt) {
  // Reverse through zero first; no discontinuous jump between throttle directions.
  if(current*target<0) return approach(current, 0, decel*dt);
  return approach(current,target,(std::abs(target)>std::abs(current)?accel:decel)*dt);
}
}
