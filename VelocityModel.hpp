#include <utility>

#ifndef TPRT_VELOCITYMODEL_HPP
#define TPRT_VELOCITYMODEL_HPP

#include "Horizon/FlatHorizon.hpp"
#include "Layer.hpp"
#include <array>
#include <limits>
#include <memory>
#include <vector>
namespace ray_tracing {
class VelocityModel {
private:
  std::vector<std::unique_ptr<Layer>> layers;

  // по оси z
  // ось направлена вниз
  static std::unique_ptr<FlatHorizon> getUpperHorizon() {
    std::vector<float> anchor = {0, 0, 0};
    float depth = (float)INT_MAX;
    float dep = 0.0f;
    float azimut = 0.0f;
    std::string name = "upper";
    std::array<float, 2> up_region = {10000.0f, 10000.0f}; // TODO: correct it
    std::array<float, 2> min_region = {-10000.0f, -10000.0f};
    std::vector<std::array<float, 2>> region;
    region.push_back(up_region);
    region.push_back(min_region);
    return std::make_unique<FlatHorizon>(depth, dep, azimut, region, anchor,
                                         name);
  }

  static std::unique_ptr<FlatHorizon> getLowerHorizon() {
    std::vector<float> anchor = {0, 0, 0};
    float depth = (float)INT_MIN;
    float dep = 0.0f;
    float azimut = 0.0f;
    std::string name = "lower";
    std::array<float, 2> up_region = {10000.0f, 10000.0f}; // TODO: correct it
    std::array<float, 2> min_region = {-10000.0f, -10000.0f};
    std::vector<std::array<float, 2>> region;
    region.push_back(up_region);
    region.push_back(min_region);
    return std::make_unique<FlatHorizon>(depth, dep, azimut, region, anchor,
                                         name);
  }

public:
  Layer *getLayer(int index) {
    assert(index < layers.size());
    return layers.at(index).get();
  }

  std::vector<Layer *> getLayers() {
    std::vector<Layer *> layers_ptr;
    for (auto &layer : layers) {
      layers_ptr.push_back(layer.get());
    }
    return layers_ptr;
  }

  int getLayersCount() { return layers.size(); }

  VelocityModel(std::vector<std::unique_ptr<Layer>> _layers) {
    /*std::cerr << "VelocityModelInit: layers size is " << _layers.size()
              << "\n"; // TODO: delete or #ifdef debug*/
    layers = std::move(_layers);
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
