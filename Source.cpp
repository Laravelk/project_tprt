//
// Created by Алексей Матвеев on 06.06.2018.
//

#include <vector>
#include "Source.hpp"

namespace ray_tracing {
    const std::array<float, 3> & Source::getLocation() const {
        return location;
    }

    rapidjson::Document Source::toJSON() {
        rapidjson::Document doc;
        rapidjson::Value json_val;
        rapidjson::Value tmp_json_val;
        doc.SetObject();

        auto &allocator = doc.GetAllocator();

        json_val.SetArray().PushBack(location[0], allocator).
                PushBack(location[1], allocator).
                PushBack(location[2], allocator);
        doc.AddMember("Location", json_val, allocator);

        json_val.SetArray();
        for (int i = 0; i < 3; i++) {
            tmp_json_val.SetArray();
            tmp_json_val.PushBack(moment[i][0], allocator).
                    PushBack(moment[i][1], allocator).
                    PushBack(moment[i][2], allocator);

            json_val.PushBack(tmp_json_val, allocator);
        }
        doc.AddMember("Moment", json_val, allocator);

        json_val.SetFloat(magnitude);
        doc.AddMember("Magnitude", json_val, allocator);

        json_val.SetString("END", allocator);
        doc.AddMember("Cardinal", json_val, allocator);

        json_val.SetFloat(t0);
        doc.AddMember("T0", json_val, allocator);

        json_val.SetString(type.c_str(), allocator);
        doc.AddMember("SType", json_val, allocator);
        return doc;
    }

    Source Source::fromJSON(const rapidjson::Value &doc) {
        if (!doc.IsObject())
            throw std::runtime_error("Source::fromJSON() - document should be an object");

        std::vector<std::string> required_fields = {"Location", "Moment", "Cardinal", "Magnitude", "T0", "SType"};

        for (const auto &field: required_fields) {
            if (!doc.HasMember(field.c_str()))
                throw std::runtime_error("Source::fromJSON() - invalid JSON, missing field " + field);
        }

        if (!doc["Location"].IsArray())
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `Location` should be an array");

        if (doc["Location"].Size() != 3)
            throw std::runtime_error("Source::fromJSON - invalid JSON, wrong 'Location' size (should be equal three)");

        if (!doc["Moment"].IsArray())
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `Orientation` should be an array");

        if (doc["Moment"].Size() != 3)
            throw std::runtime_error("Source::fromJSON - invalid JSON, wrong 'Orientation' size (should be equal Nc)");

        if (!doc["Cardinal"].IsString())
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `Cardinal` should be a string");

        if (!doc["Magnitude"].IsFloat())
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `Magnitude` should be a float");

        if (!doc["T0"].IsFloat())
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `T0` should be a float");

        if (!doc["SType"].IsString())
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `SType` should be a string");

        float magnitude = doc["Magnitude"].GetFloat();
        float t0 = doc["T0"].GetFloat();
        std::string stype = doc["SType"].GetString();
        std::string cardinal = doc["Cardinal"].GetString();

        if (cardinal != "END")
            throw std::runtime_error("Source::fromJSON() - invalid JSON, `Cardinal` should be equal 'END'");

        std::array<float, 3> location{doc["Location"][0].GetFloat(), doc["Location"][1].GetFloat(),
                                      doc["Location"][2].GetFloat()};

        std::array<std::array<float, 3>, 3> moment{};

        for (uint64_t i = 0; i < 3; i++) {
            if (!doc["Moment"][i].IsArray())
                throw std::runtime_error("Source::fromJSON() - invalid JSON, `Moment` should be an 2D array");
            moment[i] = {doc["Moment"][i][0].GetFloat(), doc["Moment"][i][1].GetFloat(),
                         doc["Moment"][i][2].GetFloat()};
        }

        return Source(location, moment, magnitude, t0, stype);
    }
}
