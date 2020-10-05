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

  for (const auto &layer : layers) {
    tmp_json_val.CopyFrom(layers[0].get()->toJSON(), allocator);
    doc.PushBack(tmp_json_val, allocator);
  }

  // doc.AddMember("Velocity model", json_val, allocator);

  return doc;
}

std::unique_ptr<VelocityModel>
VelocityModel::fromJSON(const rapidjson::Value &doc) {
  if (!doc.IsArray())
    throw std::runtime_error(
        "Velocity model::fromJSON() - document should be an array");

  return std::make_unique<VelocityModel>(doc);
}
} // namespace ray_tracing
