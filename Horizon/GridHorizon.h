#ifndef TPRT_GRIDHORIZON_H
#define TPRT_GRIDHORIZON_H

// Created by Иван Морозов on 2020-06-17.

#include <array>
#include <string>
#include "Horizon.h"
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"

namespace ray_tracing {
    class GridHorizon : public Horizon {
    private:
        float azimuth;

        _2D::BicubicInterpolator<float> interpolator;
        std::array<float, 3> anchor;
        std::vector<std::array<float, 3>> normal;
        std::string name;

        std::vector<std::tuple<float, float, float>> points;

        /*
        * anchor: first point of horizon
        * normal: vector of normal vectors for all points
        * name: name of horizon
        * points: all points after interpolation
         */
        GridHorizon(std::array<float, 3>, std::vector<std::array<float, 3>>, std::string,
                std::vector<std::tuple<float, float, float>>);

        static bool checkGrid(std::vector<double> &, std::vector<double> &,
                std::vector<double> &, double);

        /*
         * x: x cord
         * y: y cord
         * @return gradient in (x,y)
         * */
        std::array<float, 2> calculateGradientInPoint(float x, float y) const;
        std::array<float, 3> normalAtPoint(float x, float y, float z) const;

        constexpr static double EPS = 0.000001;
    public:
        // static method
        // points: vector of points
        // step: grid step
        // @return _2D::BicubicInterpolator
        static _2D::BicubicInterpolator<float> st_interpolation
                (const std::vector<std::tuple<float, float, float>>& points);

        // non-static method
        // points: vector of points
        // step: grid step
        // @return true if interpolation success else return false
        bool interpolation(const std::vector<std::tuple<float, float, float>>& points);

        virtual rapidjson::Document toJSON() override;

        float getDepth(std::array<float, 2> cord) const override;
        float getDepth(float x, float y) const;

        /* return source points */
        std::vector<std::tuple<float, float, float>> get_points() const {
            return points;
        }


        /* calculate intersect
         * x0: first point (source)
         * x1: second point (receiver)
         * @return interest point */
        std::array<float, 3>
        calcIntersect(const std::array<float, 3> &x0, const std::array<float, 3> &x1) const override;

        /* get GridHorizon from JSON file */
        static GridHorizon fromJSON(const rapidjson::Value &doc);

        /* @return z value for x, y */
        double operator()(float x, float y) const;
    };
}

#endif //TPRT_GRIDHORIZON_H
