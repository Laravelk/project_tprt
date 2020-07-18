// Created by Иван Морозов on 2020-07-15.

#ifndef TPRT_DERIVATIVE_H
#define TPRT_DERIVATIVE_H

#include <array>
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"

class Derivative {
private:
    constexpr static double EPS = 0.0001;
    Derivative() {}

public:
    /*
     * x: x cord of point
     * y: y cord of point
     * interpolator: BicubicInterpolator from libInterpolate
     * @return first derivative (dz/dx) */
    static float derivative_x(float x, float y,
            const _2D::BicubicInterpolator<float>& interpolator) {
        return (interpolator(x + EPS, y) - interpolator(x, y)) / EPS;
    }

    /*
     * x: x cord of point
     * y: y cord of point
     * interpolator: BicubicInterpolator from libInterpolate
     * @return first derivative (dz/dy) */
    static float derivative_y(float x, float y,
            const _2D::BicubicInterpolator<float>& interpolator) {
        return (interpolator(x, y + EPS) - interpolator(x,y)) / EPS;
    }
};

#endif //TPRT_DERIVATIVE_H
