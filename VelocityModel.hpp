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
private:
  std::vector<std::unique_ptr<Layer>> layers;

public:
  Layer *getLayer(int index) {
    assert(index < layers.size());
    return layers.at(index).get();
  }

  VelocityModel(const rapidjson::Value &doc) {
    uint64_t layer_number = doc.Size();
    for (uint64_t i = 0; i < layer_number; i++) { // TODO: 1 -> layer_number
      layers.push_back(Layer::fromJSON(doc[i]));
    }
  }

  rapidjson::Document toJSON();

  static std::unique_ptr<VelocityModel> fromJSON(const rapidjson::Value &doc);
};
} // namespace ray_tracing

#endif // TPRT_VELOCITYMODEL_HPP
