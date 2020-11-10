// Created by Иван Морозов on 2020-06-17.

#include "../Derivative.h"
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <iostream>
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

  json_val.SetString(this->getName().c_str(), allocator);
  doc.AddMember("Name", json_val, allocator);

  return doc;
}

float GridHorizon::getDepth(std::array<float, 2> cord) const {
  return interpolator(cord.at(0), cord.at(1));
}

std::vector<float>
GridHorizon::calcIntersect(const std::array<float, 3> &x0,
                           const std::array<float, 3> &x1) const {
  std::array<float, 3> vector = {x1[0] - x0[0], x1[1] - x0[1],
                                 x1[2] - x0[2]}; // vector from x0 to x1

  if ((x0[2] - getDepth(x0[0], x0[1])) * (x1[2] - getDepth(x1[0], x1[1])) >
      0) { // if point x1 & x2 in one side from horizon
    return {};
  }

  std::vector<float> intersect = minimize(x0, x1, vector);

  return intersect;
}

std::array<double, 2>
GridHorizon::getGradientInPoint(std::array<double, 2> cord) const {
  return getGradientInPoint(cord[0], cord[1]);
}

std::array<double, 2> GridHorizon::getGradientInPoint(double x,
                                                      double y) const {
  double derivative_x = 0, derivative_y = 0;
  double EPS = gradient_step;
  double xEPS = x + EPS;
  double yEPS = y + EPS;

  double t1 = getDepth(xEPS, y);
  double t2 = getDepth(x, y);

  derivative_x = (getDepth(xEPS, y) - getDepth(x, y)) / EPS;
  derivative_y = (getDepth(x, yEPS) - getDepth(x, y)) / EPS;

  std::vector<float> d = calculateGradientInPoint(x, y); // TODO: delete

  return {derivative_x, derivative_y};
}

/*
 * doc: "top" in json file
 * @return: GridHorizon object
 * */
std::unique_ptr<GridHorizon>
GridHorizon::fromJSON(const rapidjson::Value &doc) {
  if (!doc.IsObject()) {
    throw std::runtime_error(
        "GridHorizon::fromJSON() - document should be an object");
  }

  std::vector<std::string> required_fields = {"Dip", "Points", "Anchor",
                                              "Cardinal", "Name"};

  if (!doc["Dip"].IsFloat())
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, `Dip` should be a float");

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

  std::string cardinal = doc["Cardinal"].GetString();

  if (cardinal != "END")
    throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be equal 'END'");

  std::string name = doc["Name"].GetString();
  std::cerr << "GridHorizon::fromJSON:name " << name
            << "\n"; // TODO: remove or #ifdef
  std::vector<float> anchor{doc["Anchor"][0].GetFloat(),
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

  return std::make_unique<GridHorizon>(name, points);
}

GridHorizon::GridHorizon(std::string _name,
                         std::vector<std::tuple<float, float, float>> _points)
    : points(_points) {
  name = _name;
  find_corner(); // TODO: delete
  interpolation(points);
}

/* check grid value */
_2D::BicubicInterpolator<float> GridHorizon::getInterpolator() const {
  return interpolator;
}

void GridHorizon::setInterpolator(
    const _2D::BicubicInterpolator<float> &value) {
  interpolator = value;
}

std::vector<std::tuple<float, float, float>> GridHorizon::getPoints() const {
  return points;
}

void GridHorizon::setPoints(
    const std::vector<std::tuple<float, float, float>> &value) {
  points = value;
}

bool GridHorizon::checkGrid(std::vector<float> &x, std::vector<float> &y,
                            std::vector<float> &z) {
  if (x.size() != y.size() && (y.size() != z.size())) {
    return false;
  }

  for (unsigned long i = 0; i < x.size(); i++) {
    float x_value = x.at(i);
    float y_value = y.at(i);

    float inter = interpolator(x_value, y_value);
    float test_inter = interpolator(60000, y_value);

    //    std::cerr << x_value << ", " << y_value << ": " << z[i] << " = " <<
    //    inter
    //              << " : " << test_inter << std::endl;
  }
  return true;
}

void GridHorizon::find_corner() {
  for (auto it : points) {
    float x = std::get<0>(it);
    float y = std::get<1>(it);
    if (x < left_top.at(0) && y > left_top.at(1)) {
      left_top = {x, y};
    }
    if (x > right_top.at(0) && y > right_top.at(1)) {
      right_top = {x, y};
    }
    if (x < left_bottom.at(0) && y < left_bottom.at(1)) {
      left_bottom = {x, y};
    }
    if (x > right_bottom[0] && y < right_bottom[1]) {
      right_bottom = {x, y};
    }
  }
  return;
}

bool GridHorizon::interpolation(
    const std::vector<std::tuple<float, float, float>> &points_array) {
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;

  const int MIN_POINTS_COUNT = 4;

  assert(points_array.size() >= MIN_POINTS_COUNT); //

  x.reserve(points_array.size());
  y.reserve(points_array.size());
  z.reserve(points_array.size());

  gradient_step = abs(x[1] - x[0]);

  for (auto point : points_array) {
    x.push_back(std::get<0>(point));
    y.push_back(std::get<1>(point));
    z.push_back(std::get<2>(point));
  }

  interpolator.setData(x, y, z);
  // checkGrid(x, y, z);
  return true;
}

double GridHorizon::operator()(float x, float y) const {
  return interpolator(x, y);
}

Horizon *GridHorizon::clone() {
  GridHorizon *new_horizon = new GridHorizon(*this);
  return new_horizon;
}

float GridHorizon::getDepth(float x, float y) const {
  //  std::cerr << "GridHorizon::getDepth: " << x << " " << y << " "
  //            << interpolator(x, y) << std::endl;
  return interpolator(x, y);
}

std::vector<float> GridHorizon::calculateGradientInPoint(float x,
                                                         float y) const {
  std::vector<float> gradient = {Derivative::derivative_x(x, y, interpolator),
                                 Derivative::derivative_y(x, y, interpolator)};
  return gradient;
}

std::vector<float> GridHorizon::normalAtPoint(float x, float y, float z) const {
  std::vector<float> gradient =
      calculateGradientInPoint(x, y); // [dz/dx, dz/dy]
  float norm = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] + 1);
  return {gradient[0] / norm, gradient[1] / norm,
          1}; // [dz/dx, dy/dz, 1], unit normal
}

std::vector<float>
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
