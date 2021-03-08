#ifndef TPRT_HORIZON_H
#define TPRT_HORIZON_H

// Created by Иван Морозов on 2020-06-17.

#include <array>
#include <string>

#include "../rapidjson/pointer.h"

namespace ray_tracing {
class Horizon {
public:
  Horizon() = default;
  Horizon(Horizon &hor) = default;
  virtual ~Horizon() = default;
  virtual rapidjson::Document toJSON() = 0;

  virtual float getDepth(std::array<float, 2> x) const = 0;

  virtual std::array<double, 2> getGradientInPoint(double x,
                                                   double y) const = 0;
  virtual std::array<double, 2>
  getGradientInPoint(std::array<double, 2> cord) const = 0;

  const std::string getName() const { return name; }
  void setName(std::string new_name) { name = new_name; }

protected:
  std::string name;
};
} // namespace ray_tracing

#endif // TPRT_HORIZON_H
