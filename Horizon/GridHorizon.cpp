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

  std::vector<std::string> required_fields = {"Points", "Cardinal", "Name"};

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

  std::vector<std::tuple<float, float, float>> points;

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
  interpolation(points);
}

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
  const int MIN_POINTS_COUNT = 4;

  assert(points_array.size() >= MIN_POINTS_COUNT);
  long size = points_array.size();
  _2D::BicubicInterpolator<float>::VectorType xx(size), yy(size), zz(size);

  for (long i = 0; i < size; i++) {
    xx(i) = std::get<0>(points_array[i]);
    yy(i) = std::get<1>(points_array[i]);
    zz(i) = std::get<2>(points_array[i]);
  }

  interpolator.setData(xx, yy, zz);
  return true;
}

double GridHorizon::operator()(float x, float y) const {
  return interpolator(x, y);
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
} // namespace ray_tracing
