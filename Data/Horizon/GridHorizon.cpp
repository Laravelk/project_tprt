// Created by Иван Морозов on 2020-06-17.

#include "../../Math/Derivative.h"
#include "libInterpolate/Interpolators/_2D/BicubicInterpolator.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <tuple>
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

std::array<float, 2>
GridHorizon::getGradientInPoint(std::array<double, 2> cord) const {
  return getGradientInPoint(cord[0], cord[1]);
}

std::array<float, 2> GridHorizon::getGradientInPoint(double x,
                                                      double y) const {
  return calculateGradientInPoint(x, y);
}

std::unique_ptr<GridHorizon>
GridHorizon::fromJSON(const rapidjson::Value &doc) {
  if (!doc.IsObject()) {
    throw std::runtime_error(
        "GridHorizon::fromJSON() - document should be an object");
  }

  std::vector<std::string> required_fields = {"PointsFileName", "Cardinal", "Name"};

  if (!doc["Cardinal"].IsString())
    throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be a string");

  if (!doc["Name"].IsString())
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, `Name` should be a string");

  if (!doc["PointsFileName"].IsString()) {
    throw std::runtime_error(
        "GridHorizon::fromJSON() - invalid JSON, 'FilePath "
        "String' should be a string");
  }

  std::string cardinal = doc["Cardinal"].GetString();

  if (cardinal != "END")
    throw std::runtime_error("GridHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be equal 'END'");

  std::string name = doc["Name"].GetString();

  std::vector<std::tuple<float, float, float>> points;

  std::ifstream pointsFile(doc["PointsFileName"].GetString());
    std::cerr << "Open file: " << doc["PointsFileName"].GetString() << std::endl;
  while (!pointsFile.eof()) {
    float x;
    float y;
    float z;
    pointsFile >> x;
    pointsFile >> y;
    pointsFile >> z;
    auto tuple = std::make_tuple(x, y, z);
    points.push_back(tuple);
  }

  pointsFile.close();

  std::vector<std::array<float, 2>> region;

  for (SizeType i = 0; i < doc["Region"].Size(); i++) {
    std::array<float, 2> point = {doc["Region"][i][0].GetFloat(),
                                  doc["Region"][i][1].GetFloat()};
    region.emplace_back(point);
  }

  return std::make_unique<GridHorizon>(name, points, region);
}

GridHorizon::GridHorizon(std::string _name,
                         std::vector<std::tuple<float, float, float>> _points,
                         std::vector<std::array<float, 2>> _region)
    : points(std::move(std::move(_points))), region(std::move(_region)) {
  name = std::move(_name);
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

bool GridHorizon::checkGrid(std::vector<float> &x, std::vector<float> &y,
                            std::vector<float> &z) {
  if (x.size() != y.size() && (y.size() != z.size())) {
    return false;
  }

  return true;
}

bool GridHorizon::interpolation(const std::vector<std::tuple<float, float, float>> &points_array) {
  const int MIN_POINTS_COUNT = 4;
  assert(points_array.size() >= MIN_POINTS_COUNT);

  long size = points_array.size();
  _2D::BicubicInterpolator<float>::VectorType xx(size), yy(size), zz(size);


  for (long i = 0; i < size; i++) {
    xx(i) = (int)std::get<0>(points_array[i]);
    yy(i) = (int)std::get<1>(points_array[i]);
    zz(i) = (int)std::get<2>(points_array[i]);
  }

  interpolator.setData(xx, yy, zz);

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
  return {
      Derivative::derivative_x(x, y, interpolator),
      Derivative::derivative_y(x, y, interpolator)
  };
}

    std::vector<float> GridHorizon::getNormal(std::array<float, 2> cord) const {
    std::array<float, 2> grad = calculateGradientInPoint(cord[0], cord[1]);
    std::vector<float> n = { grad[0], grad[1], -1 };
    float norm = pow(grad[0] * grad[0] + grad[1] * grad[1] + 1,2);
    for (int i = 0; i < 3; i++) {
        n[i] = n[i] / norm;
    }
    return n;
    }

    Eigen::Matrix2f GridHorizon::getHessian(const float x, const float y) const {
        return Derivative::calculate_hessian(x, y, interpolator);
    }
} // namespace ray_tracing
