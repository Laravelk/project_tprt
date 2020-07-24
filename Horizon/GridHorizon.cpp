// Created by Иван Морозов on 2020-06-17.

#include "../Derivative.h"
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <memory>
#include <utility>
#include <vector>

#include "GridHorizon.h"

using namespace rapidjson;

namespace ray_tracing {
rapidjson::Document GridHorizon::toJSON() {
  rapidjson::Document doc;
  rapidjson::Value json_val;
  rapidjson::Value tmp_json_val;
  doc.SetObject();

  auto &allocator = doc.GetAllocator();

  json_val.SetString("grid", allocator);
  doc.AddMember("HType", json_val, allocator);

  json_val.SetFloat(azimuth);
  doc.AddMember("Azimuth", json_val, allocator);

  json_val.SetArray();
  json_val.PushBack(anchor[0], allocator).PushBack(anchor[1], allocator);
  doc.AddMember("Anchor", json_val, allocator);

  // array of arrays? TODO: do
  for (auto &point : points) {
    json_val.SetArray();
    tmp_json_val.SetArray();
    tmp_json_val.PushBack(std::get<0>(point), allocator)
        .PushBack(std::get<1>(point), allocator)
        .PushBack(std::get<2>(point), allocator);
    json_val.PushBack(tmp_json_val, allocator);
  }
  doc.AddMember("Points", json_val, allocator);

  json_val.SetString("END", allocator);
  doc.AddMember("Cardinal", json_val, allocator);

  json_val.SetString(name.c_str(), allocator);
  doc.AddMember("Name", json_val, allocator);

  return doc;
}

float GridHorizon::getDepth(std::array<float, 2> cord) const {
  return interpolator(cord.at(0), cord.at(1));
}

std::array<float, 3>
GridHorizon::calcIntersect(const std::array<float, 3> &x0,
                           const std::array<float, 3> &x1) const {
  std::array<float, 3> vector = {x1[0] - x0[0], x1[1] - x0[1],
                                 x1[2] - x0[2]}; // vector from x0 to x1

  if ((x0[2] - getDepth(x0[0], x0[1])) * (x1[2] - getDepth(x1[0], x1[1])) >
      0) { // if point x1 & x2 in one side from horizon
    return {};
  }

  std::array<float, 3> intersect = minimize(x0, x1, vector);

  return intersect;
}

/*
 * doc: "top" in json file
 * @return: GridHorizon object
 * */
std::shared_ptr<GridHorizon>
GridHorizon::fromJSON(const rapidjson::Value &doc) {
  if (!doc.IsObject()) {
    throw std::runtime_error(
        "GridHorizon::fromJSON() - document should be an object");
  }

  std::vector<std::string> required_fields = {"Dip",    "Azimuth",  "Points",
                                              "Anchor", "Cardinal", "Name"};

  if (!doc["Dip"].IsFloat())
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, `Dip` should be a float");

  if (!doc["Azimuth"].IsFloat())
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, `Azimuth` should be a float");

  if (!doc["Anchor"].IsArray())
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, `Anchor` should be an array");

  if (doc["Anchor"].Size() != 3)
    throw std::runtime_error("GridHorizon::fromJSON - invalid JSON, wrong "
                             "'Anchor' size (should be equal three)");

  if (!doc["Cardinal"].IsString())
    throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be a string");

  if (!doc["Name"].IsString())
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, `Name` should be a string");

  if (!doc["Points"].IsArray()) {
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, 'Array' should be a array");
  }

  float dip = doc["Dip"].GetFloat();
  float azimuth = doc["Azimuth"].GetFloat();
  std::string cardinal = doc["Cardinal"].GetString();

  if (cardinal != "END")
    throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be equal 'END'");

  std::string name = doc["Name"].GetString();
  std::array<float, 3> anchor{doc["Anchor"][0].GetFloat(),
                              doc["Anchor"][1].GetFloat(),
                              doc["Anchor"][2].GetFloat()};

  std::vector<std::tuple<float, float, float>> points;
  std::vector<std::array<float, 3>> normal;
  std::array<float, 3> norm = {3, 3, 3};
  normal.push_back(norm);

  for (SizeType i = 0; i < doc["Points"].Size(); i++) {
    points.emplace_back(doc["Points"][i][0].GetFloat(),
                        doc["Points"][i][1].GetFloat(),
                        doc["Points"][i][2].GetFloat());
  }

  return std::make_shared<GridHorizon>(anchor, normal, name, points);
}

GridHorizon::GridHorizon(std::array<float, 3> _anchor,
                         std::vector<std::array<float, 3>> _normal,
                         std::string _name,
                         std::vector<std::tuple<float, float, float>> _points)
    : anchor(_anchor), normal(_normal), name(_name), points(_points) {
  interpolation(points);
}

//    _2D::BicubicInterpolator<float>
//    GridHorizon::st_interpolation(const std::vector<std::tuple<float, float,
//    float>>& points_array) {
//        // points -> x, y, z. point<x,y,z> in points
//
//        std::vector<std::tuple<float, float, float>> new_points;
//
//        std::vector<double> x;
//        std::vector<double> y;
//        std::vector<double> z;
//
//        x.reserve(points_array.size());
//        y.reserve(points_array.size());
//        z.reserve(points_array.size());
//
//        _2D::BicubicInterpolator<float> inter; // interpolator
//        for (auto point : points_array) {
//            x.push_back(std::get<0>(point));
//            y.push_back(std::get<1>(point));
//            z.push_back(std::get<2>(point));
//        }
//
//        inter.setData(x, y, z);
//
//        return inter;
//    }

/* check grid value */
bool GridHorizon::checkGrid(std::vector<double> &x, std::vector<double> &y,
                            std::vector<double> &z, const double step) {
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

bool GridHorizon::interpolation(
    const std::vector<std::tuple<float, float, float>> &points_array) {
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;

  x.reserve(points_array.size());
  y.reserve(points_array.size());
  z.reserve(points_array.size());

  for (auto point : points_array) {
    x.push_back(std::get<0>(point));
    y.push_back(std::get<1>(point));
    z.push_back(std::get<2>(point));
    //    std::cerr << std::get<0>(point) << " " << std::get<1>(point) << " "
    //              << std::get<2>(point) << std::endl;
  }

  interpolator.setData(x, y, z);
  return true;
}

double GridHorizon::operator()(float x, float y) const {
  return interpolator(x, y);
}

float GridHorizon::getDepth(float x, float y) const {
  return interpolator(x, y);
}

std::array<float, 2> GridHorizon::calculateGradientInPoint(float x,
                                                           float y) const {
  std::array<float, 2> gradient = {
      Derivative::derivative_x(x, y, interpolator),
      Derivative::derivative_y(x, y, interpolator)};
  return gradient;
}

std::array<float, 3> GridHorizon::normalAtPoint(float x, float y,
                                                float z) const {
  std::array<float, 2> gradient =
      calculateGradientInPoint(x, y); // [dz/dx, dz/dy]
  float norm = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] + 1);
  return {gradient[0] / norm, gradient[1] / norm,
          1}; // [dz/dx, dy/dz, 1], unit normal
}

std::array<float, 3>
GridHorizon::minimize(const std::array<float, 3> &x0,
                      const std::array<float, 3> &x1,
                      const std::array<float, 3> &vector) const {
  /* function zone borders */
  float left_border_x = 0;
  float right_border_x = 0;
  float left_border_y = 0;
  float right_border_y = 0;

  /* defined start boundaries of the function */
  left_border_x = x0[0] > x1[0] ? x1[0] : x0[0];
  right_border_x = x0[0] > x1[0] ? x0[0] : x1[0];
  left_border_y = x0[1] > x1[1] ? x1[1] : x0[1];
  right_border_y = x0[1] > x1[1] ? x0[1] : x1[1];

  /* s: way path from x0 to x1 */
  float step = 0.0002f;
  float s = 0.0f;

  float prev_z_line = x0[2] + s * vector[2];

  for (unsigned long i = 0; i < 1.0f / step; i++) {
    float z_inter = interpolator(x0[0] + s * vector[0], x0[1] + s * vector[1]);
    float z_line = x0[2] + s * vector[2];
    if ((z_inter - z_line) * (z_inter - prev_z_line) <= 0) {
      return {(x0[0] + (s - step / 2) * vector[0]),
              (x0[1] + (s - step / 2) * vector[1]), z_inter};
    }
    prev_z_line = z_line;
    s += step;
  }
  return {0, 0, 0};
}
} // namespace ray_tracing
