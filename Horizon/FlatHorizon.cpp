#include "FlatHorizon.hpp"
#include <iostream>
#include <memory>
#include <vector>

/*
 * граница между средами
 *
 */
namespace ray_tracing {

FlatHorizon::FlatHorizon(float depth, float dip, float azimuth,
                         std::vector<std::array<float, 2>> _region,
                         std::vector<float> anchor, std::string _name)
    : dip(dip), azimuth(azimuth), depth((-1.0f) * depth), anchor(anchor),
      region(_region) {

  name = _name;
  normal.push_back(sin(dip * M_PI / 180) * cos(azimuth * M_PI / 180));
  normal.push_back((dip * M_PI / 180) * sin(azimuth * M_PI / 180));
  normal.push_back((-1.0f) * cos(dip * M_PI / 180));
  D = (-1.0f) *
      (normal[0] * anchor[0] + normal[1] * anchor[1] + normal[2] * depth);
}

float FlatHorizon::getDepth(std::array<float, 2> x) const {
  return (-1.0f) * (normal[0] * x[0] + normal[1] * x[1] + D) /
         (normal[2] + 1e-16);
}

void FlatHorizon::setDepth(float value) { depth = value; }

float FlatHorizon::getDip() const { return dip; }

void FlatHorizon::setDip(float value) { dip = value; }

float FlatHorizon::getAzimuth() const { return azimuth; }

void FlatHorizon::setAzimuth(float value) { azimuth = value; }

std::vector<float> FlatHorizon::getAnchor() const { return anchor; }

void FlatHorizon::setAnchor(const std::vector<float> &value) { anchor = value; }

std::vector<float> FlatHorizon::getNormal(std::array<float, 2> cord = {0, 0}) const { return normal; }

void FlatHorizon::setNormal(const std::vector<float> &value) { normal = value; }

std::array<double, 2>
FlatHorizon::getGradientInPoint(std::array<double, 2> cord) const {
  return getGradientInPoint(cord[0], cord[1]);
}

// TODO: сменить название
std::array<double, 2> FlatHorizon::getGradientInPoint(double x,
                                                      double y) const {
  return {(-1) * normal[0] / normal[2], (-1) * normal[1] / normal[2]};
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
      "Dip", "Azimuth", "Depth", "Anchor", "Cardinal", "Name", "Region"};


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

  for (rapidjson::SizeType i = 0; i < doc["Region"].Size(); i++) {
    std::array<float, 2> point = {doc["Region"][i][0].GetFloat(),
                                  doc["Region"][i][1].GetFloat()};
    region.emplace_back(point);
  }

  return std::make_unique<FlatHorizon>(depth, dip, azimuth, region, anchor);
}

} // namespace ray_tracing
