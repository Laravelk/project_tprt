#include <utility>

#ifndef TPRT_VELOCITYMODEL_HPP
#define TPRT_VELOCITYMODEL_HPP

/*
 *
 *
 * */

#include "Layer.hpp"
#include <vector>
namespace ray_tracing {
class VelocityModel {
  std::vector<Layer> layers;

public:
  VelocityModel(std::vector<Layer> layers) : layers(std::move(layers)) {}

  const std::vector<Layer> &getLayers() const;
  std::vector<Layer> &getLayers();

  rapidjson::Document toJSON();

  static VelocityModel fromJSON(const rapidjson::Value &doc);
};
} // namespace ray_tracing

#endif // TPRT_VELOCITYMODEL_HPP
