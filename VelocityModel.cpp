#include "VelocityModel.hpp"

// VelocityModel::VelocityModel(const std::vector<Layer> layers)

namespace ray_tracing {
rapidjson::Document VelocityModel::toJSON() {
  rapidjson::Document doc;
  rapidjson::Value json_val;
  rapidjson::Value tmp_json_val;
  doc.SetObject();

  auto &allocator = doc.GetAllocator();

  doc.SetArray();

  //  for (const auto &layer : layers) {
  //    tmp_json_val.CopyFrom(layers[0].get()->toJSON(), allocator);
  //    doc.PushBack(tmp_json_val, allocator);
  //  }

  // doc.AddMember("Velocity model", json_val, allocator);

  return doc;
}

std::unique_ptr<VelocityModel>
VelocityModel::fromJSON(const rapidjson::Value &doc) {
  if (!doc.IsArray())
    throw std::runtime_error(
        "Velocity model::fromJSON() - document should be an array");

  std::vector<std::unique_ptr<Layer>> local_layers;
  uint64_t layer_number = doc.Size();
  local_layers.push_back(
      std::make_unique<Layer>(2100, 1200, 2300, getUpperHorizon(), "upper layer"));
  for (uint64_t i = 0; i < layer_number; i++) { // TODO: 1 -> layer_number
    local_layers.push_back(Layer::fromJSON(doc[i]));
  }
    local_layers.push_back(
            std::make_unique<Layer>(3500, 2200, 2700, getLowerHorizon(), "lower layer"));

  return std::make_unique<VelocityModel>(std::move(local_layers));
}
} // namespace ray_tracing
