#include "FlatHorizon.hpp"
#include <iostream>
#include <memory>
#include <vector>

/*
 * граница между средами
 * */
namespace ray_tracing {

float FlatHorizon::getDepth(std::array<float, 2> x) const {
  std::array<float, 2> absolute_x = {{x[0] - anchor[0], x[1] - anchor[1]}};
  float z = (depth - (absolute_x[0] * normal[0] + absolute_x[1] * normal[1])) /
            (normal[2] + 1e-16f);
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
                         std::vector<float> anchor, std::string _name)
    : dip(dip), azimuth(azimuth), depth(depth), anchor(anchor) {
  name = _name;
  normal.push_back(sin(dip * M_PI / 180) * cos(azimuth * M_PI / 180));
  normal.push_back((dip * M_PI / 180) * sin(azimuth * M_PI / 180));
  normal.push_back(cos(dip * M_PI / 180));
}

// две точки и пересечение отрезка между ними с плоскостью. @return точка
// пересечения
std::vector<float>
FlatHorizon::calcIntersect(const std::array<float, 3> &x0,
                           const std::array<float, 3> &x1) const {
  float d = -depth;

  if ((normal[0] * (x1[0] - x0[0]) + normal[1] * (x1[1] - x0[1]) +
       normal[2] * (x1[2] - x0[2])) == 0)
    return {{NAN, NAN, NAN}};

  float lambda =
      -(normal[0] * x0[0] + normal[1] * x0[1] + normal[2] * x0[2] + d) /
      (normal[0] * (x1[0] - x0[0]) + normal[1] * (x1[1] - x0[1]) +
       normal[2] * (x1[2] - x0[2]));

  std::vector<float> intersect = {{x0[0] + lambda * (x1[0] - x0[0]),
                                   x0[1] + lambda * (x1[1] - x0[1]),
                                   x0[2] + lambda * (x1[2] - x0[2])}};

  // photo 1

  std::vector<float> vec0{x0[0] - intersect[0], x0[1] - intersect[1],
                          x0[2] - intersect[2]};
  std::vector<float> vec1{x1[0] - intersect[0], x1[1] - intersect[1],
                          x1[2] - intersect[2]};

  if (vec0[0] * vec1[0] + vec0[1] * vec1[1] + vec0[2] * vec1[2] <= 0)
    return intersect;
  else {
    return {{NAN, NAN, NAN}};
  }
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

  std::vector<std::string> required_fields = {"Dip",    "Azimuth",  "Depth",
                                              "Anchor", "Cardinal", "Name"};

  for (const auto &field : required_fields) {
    if (!doc.HasMember(field.c_str()))
      throw std::runtime_error(
          "FlatHorizon::fromJSON() - invalid JSON, missing field " + field);
  }

  if (!doc["Dip"].IsFloat())
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Dip` should be a float");

  if (!doc["Azimuth"].IsFloat())
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Azimuth` should be a float");

  if (!doc["Depth"].IsFloat())
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Depth` should be a float");

  if (!doc["Anchor"].IsArray())
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Anchor` should be an array");

  // TODO: убрать комментарий
  // if (doc["Anchor"].Size() != 2)
  //     throw std::runtime_error("FlatHorizon::fromJSON - invalid JSON, wrong
  //     'Anchor' size (should be equal two)");

  if (!doc["Cardinal"].IsString())
    throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be a string");

  if (!doc["Name"].IsString())
    throw std::runtime_error(
        "FlatHorizon::fromJSON() - invalid JSON, `Name` should be a string");

  float dip = doc["Dip"].GetFloat();
  float azimuth = doc["Azimuth"].GetFloat();
  float depth = doc["Depth"].GetFloat();

  std::string cardinal = doc["Cardinal"].GetString();

  if (cardinal != "END")
    throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, "
                             "`Cardinal` should be equal 'END'");

  std::string name = doc["Name"].GetString();

  std::cerr << "FlatHorizon::fromJSON:name " << name << " with depth " << depth
            << "\n"; // TODO: remove or #ifdef

  std::vector<float> anchor{doc["Anchor"][0].GetFloat(),
                            doc["Anchor"][1].GetFloat()};

  return std::make_unique<FlatHorizon>(depth, dip, azimuth, anchor, name);
}
} // namespace ray_tracing
