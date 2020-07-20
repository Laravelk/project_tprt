#include "VelocityModel.hpp"

//VelocityModel::VelocityModel(const std::vector<Layer> layers)

namespace ray_tracing {
    rapidjson::Document VelocityModel::toJSON() {
        rapidjson::Document doc;
        rapidjson::Value json_val;
        rapidjson::Value tmp_json_val;
        doc.SetObject();

        auto &allocator = doc.GetAllocator();

        doc.SetArray();

        for (const auto &layer: layers) {
            tmp_json_val.CopyFrom(layers[0].toJSON(), allocator);
            doc.PushBack(tmp_json_val, allocator);
        }

        //doc.AddMember("Velocity model", json_val, allocator);

        return doc;
    }


    VelocityModel VelocityModel::fromJSON(const rapidjson::Value &doc) {
        if (!doc.IsArray())
            throw std::runtime_error("Velocity model::fromJSON() - document should be an array");

        uint64_t layer_number = doc.Size();
        std::vector<Layer> layers;

        for (uint64_t i = 0; i < 1; i++) { // TODO: 1 -> layer_number
            layers.push_back(Layer::fromJSON(doc[i]));
        }

        return VelocityModel(layers);
    }

    const std::vector<Layer> &VelocityModel::getLayers() const {
        return layers;
    }
}