// Created by Иван Морозов on 2020-07-15.

#ifndef TPRT_DERIVATIVE_H
#define TPRT_DERIVATIVE_H

#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <array>
#include <Eigen/Dense>

class Derivative {
private:
  constexpr static float EPS = 1;
  constexpr static float REVERSE_EPS_MUL_TWO = 1.0f / (2.0f * EPS);
  Derivative() {}

public:
  /*
   * x: x cord of point
   * y: y cord of point
   * interpolator: BicubicInterpolator from libInterpolate
   * @return first derivative (dz/dx) */
  static float
  derivative_x(const float x, const float y,
               const _2D::BicubicInterpolator<float> &interpolator) {
    return (interpolator(x + EPS, y) - interpolator(x - EPS, y)) * REVERSE_EPS_MUL_TWO;
  }

    static float derivative_xx(const float x, const float y, const _2D::BicubicInterpolator<float> &interpolator) {
      float fl = derivative_x(x - EPS, y, interpolator);
      float fr = derivative_x(x + EPS, y, interpolator);
      return (fr - fl) * REVERSE_EPS_MUL_TWO;
    }

    static float derivative_yy(const float x, const float y, const _2D::BicubicInterpolator<float> &interpolator) {
        float fl = derivative_y(x, y - EPS, interpolator);
        float fr = derivative_y(x, y + EPS, interpolator);
        return (fr - fl) * REVERSE_EPS_MUL_TWO;
  }

  static float derivative_xy(const float x, const float y, const _2D::BicubicInterpolator<float> &interpolator) {
      float fl = derivative_x(x, y - EPS, interpolator);
      float fr = derivative_x(x, y + EPS, interpolator);
      return (fr - fl) * REVERSE_EPS_MUL_TWO;
  }

  static float derivative_yx(const float x, const float y, const _2D::BicubicInterpolator<float> &interpolator) {
      float fl = derivative_y(x - EPS, y, interpolator);
      float fr = derivative_y(x + EPS, y, interpolator);
      return (fr - fl) * REVERSE_EPS_MUL_TWO;
  }


    /*
   * x: x cord of point
   * y: y cord of point
   * interpolator: BicubicInterpolator from libInterpolate
   * @return first derivative (dz/dy) */
  static float
  derivative_y(const float x, const float y,
               const _2D::BicubicInterpolator<float> &interpolator) {
    return (interpolator(x, y + EPS) - interpolator(x, y - EPS)) * REVERSE_EPS_MUL_TWO;
  }

  static Eigen::Matrix2f calculate_hessian(const float x, const float y, const _2D::BicubicInterpolator<float> &interpolator) {
      Eigen::Matrix2f hessians;
      hessians(0, 0) = derivative_xx(x, y, interpolator);
      hessians(0, 1) = derivative_yx(x, y, interpolator);
      hessians(1,0) = derivative_yx(x, y, interpolator);
      hessians(1,1) = derivative_yy(x, y, interpolator);
      return hessians;
  }
};

#endif // TPRT_DERIVATIVE_H
