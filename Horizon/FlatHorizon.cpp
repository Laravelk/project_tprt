#include "FlatHorizon.hpp"
#include <iostream>
#include <memory>
#include <vector>

/*
 * граница между средами
 * */
namespace ray_tracing {

// NOTE: как это работает
float FlatHorizon::getDepth(std::array<float, 2> x) const {
  std::array<float, 2> absolute_x = {{x[0] - anchor[0], x[1] - anchor[1]}};
  float z = (depth - (absolute_x[0] * normal[0] + absolute_x[1] * normal[1])) /
            (normal[2] + 1e-16f);
  //  std::cerr << "FlatHorizon::getDepth::ABS: " << absolute_x[0] << " "
  //            << absolute_x[1] << " " << z << std::endl;
  return z;
}

Horizon *FlatHorizon::clone() {
  FlatHorizon *new_horizon = new FlatHorizon(*this);
  return new_horizon;
}

void FlatHorizon::setDepth(float value) { depth = value; }

float FlatHorizon::getDip() const { return dip; }

void FlatHorizon::setDip(float value) { dip = value; }

float FlatHorizon::getAzimuth() const { return azimuth; }

void FlatHorizon::setAzimuth(float value) { azimuth = value; }

std::vector<float> FlatHorizon::getAnchor() const { return anchor; }

void FlatHorizon::setAnchor(const std::vector<float> &value) { anchor = value; }

std::vector<float> FlatHorizon::getNormal() const { return normal; }

void FlatHorizon::setNormal(const std::vector<float> &value) { normal = value; }

FlatHorizon::FlatHorizon(float depth, float dip, float azimuth,
                         std::vector<float> anchor,
                         std::vector<std::array<float, 2>> region,
                         std::string _name)
    : dip(dip), azimuth(azimuth), depth(depth), anchor(anchor) {
  this->region = region;
  this->name = _name;
  this->normal.push_back(sin(dip * M_PI / 180) * cos(azimuth * M_PI / 180));
  this->normal.push_back((dip * M_PI / 180) * sin(azimuth * M_PI / 180));
  this->normal.push_back(cos(dip * M_PI / 180));
}

std::array<double, 2>
FlatHorizon::getGradientInPoint(std::array<double, 2> cord) const {
  return getGradientInPoint(cord[0], cord[1]);
}

std::array<double, 2> FlatHorizon::getGradientInPoint(double x,
                                                      double y) const {
  double derivative_x = 0, derivative_y = 0;
  const double EPS = 1000.0f;

  double xEPS = x + EPS;
  double yEPS = y + EPS;
  std::array<float, 2> array1{(float)xEPS, (float)y};
  std::array<float, 2> array2{(float)x, (float)y};

  double t1 = getDepth(array1);
  double t2 = getDepth(array2);

  derivative_x = (getDepth(array1) - getDepth(array2)) / EPS;
  derivative_y =
      (getDepth({(float)x, (float)(yEPS)}) - getDepth({(float)x, (float)y})) /
      EPS;

  return {derivative_x, derivative_y};
}

rapidjson::Document FlatHorizon::toJSON() {
  rapidjson::Document doc;
  rapidjson::Value json_val;
  rapidjson::Value tmp_json_val;
  doc.SetObject();

  auto &allocator = doc.GetAllocator();

  json_val.SetString("flat", allocator);
  doc.AddMember("HType", json_val, allocator);

  json_val.SetFloat(dip);
  doc.AddMember("Dip", json_val, allocator);

  json_val.SetFloat(azimuth);
  doc.AddMember("Azimuth", json_val, allocator);

  json_val.SetFloat(depth);
  doc.AddMember("Depth", json_val, allocator);

  json_val.SetArray();
  json_val.PushBack(anchor[0], allocator).PushBack(anchor[1], allocator);
  doc.AddMember("Anchor", json_val, allocator);

  json_val.SetString("END", allocator);
  doc.AddMember("Cardinal", json_val, allocator);

  json_val.SetString(this->getName().c_str(), allocator);
  doc.AddMember("Name", json_val, allocator);

  return doc;
}

std::unique_ptr<FlatHorizon>
FlatHorizon::fromJSON(const rapidjson::Value &doc) {

  if (!doc.IsObject())
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - document should be an object");

  std::vector<std::string> required_fields = {
      "Dip", "Azimuth", "Depth", "Anchor", "Cardinal", "Name" /*, "Region"*/};

  for (const auto &field : required_fields) {
    if (!doc.HasMember(field.c_str()))
      throw std::runtime_error(
          "FlatHorizon::fromJSON() - invalid JSON, missing field " + field);
  }

  if (!doc["Dip"].IsFloat()) {
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Dip` should be a float");
  }

  if (!doc["Azimuth"].IsFloat()) {
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Azimuth` should be a float");
  }

  if (!doc["Depth"].IsFloat()) {
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Depth` should be a float");
  }

  if (!doc["Anchor"].IsArray()) {
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Anchor` should be an array");
  }

  if (doc["Anchor"].Size() != 3) {
    throw std::runtime_error("FlatHorizon::fromJSON - invalid JSON, wrong "
                             "'Anchor' size (should be equal two)");
  }

  if (!doc["Cardinal"].IsString()) {
    throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be a string");
  }

  if (!doc["Name"].IsString()) {
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Name` should be a string");
  }

  if (!doc["Region"].IsArray()) {
    throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, 'Region' "
                             "size should be equal two");
  }

  float dip = doc["Dip"].GetFloat();
  float azimuth = doc["Azimuth"].GetFloat();
  float depth = doc["Depth"].GetFloat();

  std::string cardinal = doc["Cardinal"].GetString();

  if (cardinal != "END")
    throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be equal 'END'");

  std::string name = doc["Name"].GetString();

  std::vector<std::array<float, 2>> region;
  region.push_back(
      {doc["Region"][0][0].GetFloat(), doc["Region"][0][1].GetFloat()});
  region.push_back(
      {doc["Region"][1][0].GetFloat(), doc["Region"][1][1].GetFloat()});

  std::vector<float> anchor{doc["Anchor"][0].GetFloat(),
                            doc["Anchor"][1].GetFloat()};

  return std::make_unique<FlatHorizon>(depth, dip, azimuth, anchor, region,
                                       name);
}
} // namespace ray_tracing
