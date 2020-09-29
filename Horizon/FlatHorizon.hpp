#include <utility>

#ifndef TPRT_FLATHORIZON_HPP
#define TPRT_FLATHORIZON_HPP

#include "../rapidjson/document.h"
#include "Horizon.h"
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

namespace ray_tracing {
class FlatHorizon : public Horizon {
private:
  float dip;
  float azimuth;
  float depth;

  std::vector<float> anchor;
  std::vector<float> normal;

public:
  FlatHorizon(float depth, float dip, float azimuth,
              std::vector<float> anchor = {0, 0, 0}, std::string name = "");

  FlatHorizon(FlatHorizon &copy)
      : dip(copy.getDip()), depth(copy.getDepthValue()),
        azimuth(copy.getAzimuth()), anchor(copy.getAnchor()),
        normal(copy.getNormal()) {
    this->name = copy.getName();
  }

  ~FlatHorizon() override {}

  virtual float getDepth(std::array<float, 2> x) const override;

  virtual Horizon *clone() override;

  virtual std::vector<float>
  calcIntersect(const std::array<float, 3> &x0,
                const std::array<float, 3> &x1) const override;

  virtual rapidjson::Document toJSON() override;

  static std::unique_ptr<FlatHorizon> fromJSON(const rapidjson::Value &doc);
  void setDepth(float value);
  float getDepthValue() { return depth; }
  float getDip() const;
  void setDip(float value);
  float getAzimuth() const;
  void setAzimuth(float value);
  std::vector<float> getAnchor() const;
  void setAnchor(const std::vector<float> &value);
  std::vector<float> getNormal() const;
  void setNormal(const std::vector<float> &value);
};

} // namespace ray_tracing

#endif // TPRT_FLATHORIZON_HPP
