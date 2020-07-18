#ifndef TPRT_GRIDHORIZON_H
#define TPRT_GRIDHORIZON_H

//
// Created by Иван Морозов on 2020-06-17.
//

#include <array>
#include <string>
#include "Horizon.h"

namespace ray_tracing {
    class GridHorizon : public Horizon {
    private:
        float azimuth;
        double step;

        std::array<float, 3> anchor;
        std::vector<std::array<float, 3>> normal;
        std::string name;

        std::vector<std::tuple<float, float, float>> points;
        std::vector<std::tuple<float, float, float>> points_after_interpolation;

        GridHorizon(std::array<float, 3>, std::vector<std::array<float, 3>>, std::string,
                std::vector<std::tuple<float, float, float>>, double);

        static bool checkGrid(std::vector<double> &, std::vector<double> &,
                std::vector<double> &, double);

        constexpr static double EPS = 0.000001;
    public:
        // static method
        // points: vector of points
        // step: grid step
        // @return array with interpolation points
        static std::vector<std::tuple<float, float, float>> st_interpolation
                (const std::vector<std::tuple<float, float, float>>& points, double step);

        // non-static method
        // points: vector of points
        // step: grid step
        // @return true if interpolation success else return false
        bool interpolation(const std::vector<std::tuple<float, float, float>>& points, double step);

        virtual rapidjson::Document toJSON() override;

        virtual float getDepth(std::array<float, 2> cord) const override;

        std::vector<std::tuple<float, float, float>> get_points() const {
            return points;
        }

        std::vector<std::tuple<float, float, float>> get_interpolated_points() const {
            return points_after_interpolation;
        }

        virtual std::array<float, 3>
        calcIntersect(const std::array<float, 3> &x0, const std::array<float, 3> &x1) const override;

        static GridHorizon fromJSON(const rapidjson::Value &doc);
    };
}

#endif //TPRT_GRIDHORIZON_H
