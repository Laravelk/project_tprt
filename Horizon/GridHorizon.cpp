// Created by Иван Морозов on 2020-06-17.

#include <utility>
#include <vector>
#include <iostream> // TODO: delete
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"

#include "GridHorizon.h"

using namespace rapidjson;

namespace ray_tracing {
    rapidjson::Document GridHorizon::toJSON() {
        return rapidjson::Document();
    }

    float GridHorizon::getDepth(std::array<float, 2> cord) const {
        return inter;
    }

    std::array<float, 3>
    GridHorizon::calcIntersect(const std::array<float, 3> &x0, const std::array<float, 3> &x1) const {
        return std::array<float, 3>();
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

        double grid_step = doc["Step"].GetDouble();

        return GridHorizon(anchor, normal, name, points, grid_step);
    }

    /*
     * anchor: first point of horizon
     * normal: vector of normal vectors for all points
     * name: name of horizon
     * points: all points after interpolation
     * step: grid step
     * */
    GridHorizon::GridHorizon(std::array<float, 3> _anchor, std::vector<std::array<float, 3>> _normal, std::string _name,
                             std::vector<std::tuple<float, float, float>> _points, double _step) : anchor(_anchor),
                             normal(std::move(_normal)), name(std::move(_name)),
                             points(std::move(_points)), step(_step)
    {
        interpolation(points, _step);
    }

    /*
     * points: non interpolation points
     * @return: points array after interpolation
     */
    std::vector<std::tuple<float, float, float>>
    GridHorizon::st_interpolation(const std::vector<std::tuple<float, float, float>>& points_array, double step) {
        // TODO: interpolation
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

        std::sort(x.begin(), x.end());
        std::sort(y.begin(), y.end());
        std::sort(z.begin(), z.end());

        interpolator.setData(x, y, z);

//        if (!checkGrid(x, y, z, step)) { // check regular
//            return new_points;
//        }

        double new_step = step / 5.0f; // boost points count in five

        unsigned long size_x = x.size() * 5;

        std::vector<double> new_x;
        std::vector<double> new_y;
        std::vector<double> new_z;
        new_x.reserve(size_x);
        new_y.reserve(size_x);
        new_z.reserve(size_x);

        for (unsigned long i = 0; i < size_x; i++) {
            for (unsigned long j = 0; j < 5; j++) {
                new_x.push_back(x.at(i) + (double)j * new_step);
                new_y.push_back(y.at(i) + (double)j * new_step);
            }
        }

        for (auto x_value : new_x) {
            for (auto y_value : new_y) {
                    std::tuple<float, float, float> point = std::make_tuple(x_value, y_value,
                                                                               interpolator(x_value, y_value));
                    new_points.push_back(point);
            }
        }
        return new_points;
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

    // change value of points_interpolation
    bool GridHorizon::interpolation(const std::vector<std::tuple<float, float, float>> & points_array, double step) {
        points_after_interpolation = st_interpolation(points_array, step);
        return !points_after_interpolation.empty();
    }
}