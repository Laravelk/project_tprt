#include <utility>

#ifndef TPRT_FLATHORIZON_HPP
#define TPRT_FLATHORIZON_HPP

#include "../rapidjson/document.h"
#include "Horizon.h"
#include <array>
#include <cmath>
#include <iostream>

namespace ray_tracing {
class FlatHorizon : public Horizon {

public:
  float dip;
  float azimuth;
  float depth;

  std::array<float, 2> anchor;
  std::array<float, 3> normal;
  std::string name;

  FlatHorizon(float depth, float dip, float azimuth,
              std::array<float, 2> anchor = {{0, 0}}, std::string name = "");

  virtual float getDepth(std::array<float, 2> x) const override;

  virtual std::array<float, 3>
  calcIntersect(const std::array<float, 3> &x0,
                const std::array<float, 3> &x1) const override;

  virtual rapidjson::Document toJSON() override;

  static std::unique_ptr<FlatHorizon> fromJSON(const rapidjson::Value &doc);
};

} // namespace ray_tracing

#endif // TPRT_FLATHORIZON_HPP
