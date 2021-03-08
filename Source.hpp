#ifndef TPRT_SOURCE_HPP
#define TPRT_SOURCE_HPP

#include "rapidjson/document.h"
#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace ray_tracing {
class Source {
  std::array<float, 3> location;
  std::array<std::array<float, 3>, 3> moment;
  float magnitude;
  float t0;

  std::string type;

public:
  Source(std::array<float, 3> location,
         std::array<std::array<float, 3>, 3> moment, float magnitude, float t0,
         std::string type = "")
      : location(location), moment(moment), magnitude(magnitude), t0(t0),
        type(type) {}

  Source(std::array<float, 3> location) : location(location) {}

  const std::array<float, 3> &getLocation() const;

  rapidjson::Document toJSON();

  static std::vector<Source> fromFile(std::ifstream file);
  static Source fromJSON(const rapidjson::Value &doc);
};
} // namespace ray_tracing

#endif // TPRT_SOURCE_HPP
