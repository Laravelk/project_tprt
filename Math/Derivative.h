// Created by Иван Морозов on 2020-07-15.

#ifndef TPRT_DERIVATIVE_H
#define TPRT_DERIVATIVE_H

#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <array>

class Derivative {
private:
  constexpr static float EPS = 0.001f;
  Derivative() {}

public:
  /*
   * x: x cord of point
   * y: y cord of point
   * interpolator: BicubicInterpolator from libInterpolate
   * @return first derivative (dz/dx) */
  static float
  derivative_x(float x, float y,
               const _2D::BicubicInterpolator<float> &interpolator) {
    return (interpolator(x + EPS, y) - interpolator(x - EPS, y)) / (2 * EPS);
  }

  /*
   * x: x cord of point
   * y: y cord of point
   * interpolator: BicubicInterpolator from libInterpolate
   * @return first derivative (dz/dy) */
  static float
  derivative_y(float x, float y,
               const _2D::BicubicInterpolator<float> &interpolator) {
    return (interpolator(x, y + EPS) - interpolator(x, y - EPS)) / (2 * EPS);
  }
};

#endif // TPRT_DERIVATIVE_H
