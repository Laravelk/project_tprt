#include <utility>

#ifndef TPRT_RECIEVER_HPP
#define TPRT_RECIEVER_HPP

#include "rapidjson/document.h"
#include <array>
#include <fstream>
#include <string>
#include <vector>

namespace ray_tracing {
class Receiver {
  std::array<float, 3> location;
  std::vector<std::array<float, 3>> orientation;

  float sampling{};

  std::string name;

public:
  Receiver(std::array<float, 3> location,
           std::vector<std::array<float, 3>> orientation, float sampling,
           std::string name = "")
      : location(location), orientation(std::move(orientation)),
        sampling(sampling), name(std::move(name)) {}

  Receiver(std::array<float, 3> location) : location(location) {}

    void change_x_loc(float x) {
        location[0] = x;
    }

  static std::vector<Receiver> fromFile(std::ifstream &file);

  [[nodiscard]] const std::array<float, 3> &getLocation() const { return location; }

  rapidjson::Document toJSON();

  static Receiver fromJSON(const rapidjson::Value &doc);
};
} // namespace ray_tracing

#endif // TPRT_RECIEVER_HPP
