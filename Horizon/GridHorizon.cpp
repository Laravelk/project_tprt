// Created by Иван Морозов on 2020-06-17.

#include <utility>
#include <vector>
#include <iostream> // TODO: delete
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include "../Derivative.h"

#include "GridHorizon.h"

using namespace rapidjson;

namespace ray_tracing {
    rapidjson::Document GridHorizon::toJSON() {
        return rapidjson::Document();
    }

    float GridHorizon::getDepth(std::array<float, 2> cord) const {
        return interpolator(cord.at(0), cord.at(1));
    }

    std::array<float, 3>
    GridHorizon::calcIntersect(const std::array<float, 3> &x0, const std::array<float, 3> &x1) const {
        std::array<float, 3> inter;
        std::array<float, 3> vector = {x1[0] - x0[0], x1[1] - x0[1], x1[2] - x0[2] }; // vector from x0 to x1

        if ((x0[2] - getDepth(x0[0], x0[1])) * (x1[2] - getDepth(x1[0], x1[1])) > 0) { // if point x1 & x2 in one side from horizon
            return inter;
        }

    }

    /*
     * doc: "top" in json file
     * @return: GridHorizon object
     * */
    GridHorizon GridHorizon::fromJSON(const rapidjson::Value &doc) {
        if (!doc.IsObject()) {
            throw std::runtime_error("GridHorizon::fromJSON() - document should be an object");
        }

        std::vector<std::string> required_fields = {"HType", "Dip", "Azimuth", "Depth",
                                                    "Points" ,"Anchor", "Cardinal", "Name"};

        if (!doc["HType"].IsString())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `HType` should be a string");

        if (!doc["Dip"].IsFloat())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Dip` should be a float");

        if (!doc["Azimuth"].IsFloat())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Azimuth` should be a float");

        if (!doc["Depth"].IsFloat())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Depth` should be a float");

        if (!doc["Anchor"].IsArray())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Anchor` should be an array");

        if (doc["Anchor"].Size() != 3)
            throw std::runtime_error("GridHorizon::fromJSON - invalid JSON, wrong 'Anchor' size (should be equal three)");

        if (!doc["Cardinal"].IsString())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Cardinal` should be a string");

        if (!doc["Name"].IsString())
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Name` should be a string");

        if (!doc["Points"].IsArray()) {
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, 'Array' should be a array");
        }

        std::string htype = doc["HType"].GetString();

        //if (htype != "grid")
        //    throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `HType` should be equal 'grid'");

        float dip = doc["Dip"].GetFloat();
        float azimuth = doc["Azimuth"].GetFloat();
        float depth = doc["Depth"].GetFloat();

        std::string cardinal = doc["Cardinal"].GetString();

        if (cardinal != "END")
            throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, `Cardinal` should be equal 'END'");

        std::string name = doc["Name"].GetString();
        std::array<float, 3> anchor{doc["Anchor"][0].GetFloat(), doc["Anchor"][1].GetFloat(),
                                    doc["Anchor"][2].GetFloat()};

        std::vector<std::tuple<float, float, float>> points;
        std::vector<std::array<float, 3>> normal;

        for (SizeType i = 0; i < doc["Points"].Size(); i++) {
                points.emplace_back(doc["Points"][i][0].GetFloat(),
                                    doc["Points"][i][1].GetFloat(),
                                    doc["Points"][i][2].GetFloat());
        }

        return GridHorizon(anchor, normal, name, points);
    }

    GridHorizon::GridHorizon(std::array<float, 3> _anchor, std::vector<std::array<float, 3>> _normal, std::string _name,
                             std::vector<std::tuple<float, float, float>> _points) : anchor(_anchor),
                             normal(std::move(_normal)), name(std::move(_name)),
                             points(std::move(_points))
    {
        interpolation(points);
    }

    _2D::BicubicInterpolator<float>
    GridHorizon::st_interpolation(const std::vector<std::tuple<float, float, float>>& points_array) {
        // points -> x, y, z. point<x,y,z> in points

        std::vector<std::tuple<float, float, float>> new_points;

        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> z;

        x.reserve(points_array.size());
        y.reserve(points_array.size());
        z.reserve(points_array.size());

        _2D::BicubicInterpolator<float> interpolator; // interpolator
        for (auto point : points_array) {
            x.push_back(std::get<0>(point));
            y.push_back(std::get<1>(point));
            z.push_back(std::get<2>(point));
        }

        interpolator.setData(x, y, z);

        return interpolator;
    }

    /* check grid value */
    bool GridHorizon::checkGrid(std::vector<double> &x,
            std::vector<double> &y, std::vector<double> &z, const double step) {
        if (x.size() != y.size() && (y.size() != z.size())) {
            return false;
        }

        double prev_x_value = x.at(0);
        double prev_y_value = y.at(0);
        double prev_z_value = z.at(0);

        for (unsigned long i = 1; i < x.size(); i++) {
            double x_value = x.at(i);
            double y_value = y.at(i);
            double z_value = z.at(i);

            double diff_x = abs(x_value - prev_x_value);
            double diff_y = abs(y_value - prev_y_value);
            double diff_z = abs(z_value - prev_z_value);

            if (0 != diff_x || abs(step - diff_x) <= EPS) {
                return false;
            }

            if (0 != diff_y || abs(step - diff_y) <= EPS) {
                return false;
            }
            if (0 != diff_z || abs(step - diff_z) <= EPS) {
                return false;
            }

            prev_x_value = x_value;
            prev_y_value = y_value;
            prev_z_value = z_value;
        }
        return true;
    }

    bool GridHorizon::interpolation(const std::vector<std::tuple<float, float, float>> & points_array) {
        this->interpolator = st_interpolation(points_array);
        return true;
    }

    double GridHorizon::operator()(float x, float y) const {
        return interpolator(x, y);
    }

    float GridHorizon::getDepth(float x, float y) const {
        return interpolator(x, y);
    }

    std::array<float, 2> GridHorizon::calculateGradientInPoint(float x, float y) const {
        std::array<float, 2> gradient = {Derivative::derivative_x(x, y, interpolator),
                                         Derivative::derivative_y(x,y,interpolator)};
        return gradient;
    }

    std::array<float, 3> GridHorizon::normalAtPoint(float x, float y, float z) const {
        std::array<float, 2> gradient = calculateGradientInPoint(x, y); // [dz/dx, dz/dy]
        float norm = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] + 1);
        return {gradient[0] / norm, gradient[1] / norm, 1}; // [dz/dx, dy/dz, 1], unit normal
    }
}