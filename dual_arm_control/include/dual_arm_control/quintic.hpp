#pragma once

#include <cmath>
#include <limits>

namespace dual_arm_control
{

/// Scalar quintic polynomial p(t) on [ti, ti + T] matching position, velocity and acceleration at both ends.
/// Outside the interval the position is held at the boundary value and velocity/acceleration are zero.
class QuinticPoly
{
public:
  QuinticPoly() = default;

  QuinticPoly(double T, double pi, double pf, double ti, double vi = 0.0, double vf = 0.0, double ai = 0.0,
              double af = 0.0)
    : T_(T), ti_(ti), pi_(pi), pf_(pf)
  {
    const double Tc = (T < 10.0 * std::numeric_limits<double>::epsilon()) ? 0.001 : T;
    a0_ = pi;
    a1_ = vi;
    a2_ = ai / 2.0;
    const double b0 = pf - pi - vi * Tc - ai * Tc * Tc / 2.0;
    const double b1 = vf - vi - ai * Tc;
    const double b2 = af - ai;
    const double T2 = Tc * Tc, T3 = T2 * Tc, T4 = T3 * Tc, T5 = T4 * Tc;
    a3_ = 10.0 / T3 * b0 - 4.0 / T2 * b1 + 1.0 / (2.0 * Tc) * b2;
    a4_ = -15.0 / T4 * b0 + 7.0 / T3 * b1 - 1.0 / T2 * b2;
    a5_ = 6.0 / T5 * b0 - 3.0 / T4 * b1 + 1.0 / (2.0 * T3) * b2;
  }

  double initialTime() const { return ti_; }
  double finalTime() const { return ti_ + T_; }
  bool isCompleted(double t) const { return t >= finalTime(); }

  double position(double t) const
  {
    if (t < ti_)
      return pi_;
    if (t > finalTime())
      return pf_;
    const double tau = t - ti_;
    return a0_ + tau * (a1_ + tau * (a2_ + tau * (a3_ + tau * (a4_ + tau * a5_))));
  }

  double velocity(double t) const
  {
    if (t < ti_ || t > finalTime())
      return 0.0;
    const double tau = t - ti_;
    return a1_ + tau * (2.0 * a2_ + tau * (3.0 * a3_ + tau * (4.0 * a4_ + tau * 5.0 * a5_)));
  }

private:
  double T_ = 0.0, ti_ = 0.0, pi_ = 0.0, pf_ = 0.0;
  double a0_ = 0.0, a1_ = 0.0, a2_ = 0.0, a3_ = 0.0, a4_ = 0.0, a5_ = 0.0;
};

}  // namespace dual_arm_control
