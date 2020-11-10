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
    /*std::cerr << "getLayer: " << index << " of " << layers.size() - 1
              << std::endl; // NOTE: delete*/
    assert(index < layers.size());
    return layers.at(index).get();
  }

  VelocityModel(std::vector<std::unique_ptr<Layer>> _layers) {
    /*std::cerr << "VelocityModelInit: layers size is " << _layers.size()
              << "\n"; // TODO: delete or #ifdef debug*/
    layers = std::move(_layers);
  }

  VelocityModel(const rapidjson::Value &doc) {
    uint64_t layer_number = doc.Size();
    std::cerr << "VelocityModelInit: layer_number is " << layer_number
              << "\n"; // TODO: delete or #ifdef debug
    for (uint64_t i = 0; i < layer_number; i++) { // TODO: 1 -> layer_number
      layers.push_back(Layer::fromJSON(doc[i]));
    }
  }

  rapidjson::Document toJSON();

  static std::unique_ptr<VelocityModel> fromJSON(const rapidjson::Value &doc);
};
} // namespace ray_tracing

#endif // TPRT_VELOCITYMODEL_HPP
