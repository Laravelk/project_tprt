//
// Created by Алексей Матвеев on 10.06.2018.
//

#include <vector>
#include "Layer.hpp"
#include "Horizon/GridHorizon.h"

namespace ray_tracing {
    rapidjson::Document Layer::toJSON() {
        rapidjson::Document doc;
        rapidjson::Value json_val;
        rapidjson::Value tmp_json_val;
        doc.SetObject();

        auto &allocator = doc.GetAllocator();

        json_val.SetString("ISO", allocator);
        doc.AddMember("LType", json_val, allocator);

        json_val.SetFloat(Vp);
        doc.AddMember("Vp", json_val, allocator);

        json_val.SetFloat(Vs);
        doc.AddMember("Vs", json_val, allocator);

        json_val.CopyFrom(top.toJSON(), allocator);
        doc.AddMember("top", json_val, allocator);

        json_val.SetString(name.c_str(), allocator);
        doc.AddMember("Name", json_val, allocator);

        return doc;
    }

    Layer Layer::fromJSON(const rapidjson::Value &doc) {
        if (!doc.IsObject())
            throw std::runtime_error("FlatHorizon::fromJSON() - document should be an object");

        std::vector<std::string> required_fields = {"LType", "Vp", "Vs", "top", "Name"};

        for (const auto &field: required_fields) {
            if (!doc.HasMember(field.c_str()))
                throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, missing field " + field);
        }

        if (!doc["LType"].IsString())
            throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, `HType` should be a string");

        if (!doc["Vp"].IsFloat())
            throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, `Dip` should be a float");

        if (!doc["Vs"].IsFloat())
            throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, `Azimuth` should be a float");

        if (!doc["Name"].IsString())
            throw std::runtime_error("Receiver::fromJSON() - invalid JSON, `Name` should be a string");

        std::string ltype = doc["LType"].GetString();

        if (ltype != "ISO")
            throw std::runtime_error("FlatHorizon::fromJSON() - invalid JSON, `LType` should be equal 'ISO'");

        float vp = doc["Vp"].GetFloat();
        float vs = doc["Vs"].GetFloat();

        FlatHorizon hor = FlatHorizon::fromJSON(doc["top"]);
        GridHorizon gor = GridHorizon::fromJSON(doc["top"]);

        std::string name = doc["Name"].GetString();

        return Layer(vp, vs, hor, name);
    }
}
