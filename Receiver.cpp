#include "Receiver.hpp"
#include <iostream>

namespace ray_tracing {
    std::vector<Receiver> Receiver::fromFile(std::ifstream file) {
        std::vector<std::array<float, 3>> receivers_cord;

        while (!file.eof()) {
            float x = 0, y = 0, z = 0;
            file >> x;
            file >> y;
            file >> z;
            std::array<float, 3> location = {x, y, z};
            receivers_cord.push_back(location);
        }

        std::vector<Receiver> receivers;

        for (auto location : receivers_cord) {
            Receiver receiver(location);
            receivers.push_back(receiver);
        }

        return receivers;
    }

    rapidjson::Document Receiver::toJSON() {
        rapidjson::Document doc;
        rapidjson::Value json_val;
        rapidjson::Value tmp_json_val;
        doc.SetObject();

        auto &allocator = doc.GetAllocator();

        json_val.SetArray()
                .PushBack(location[0], allocator)
                .PushBack(location[1], allocator)
                .PushBack(location[2], allocator);

        doc.AddMember("Location", json_val, allocator);

        json_val.SetUint64(orientation.size());
        doc.AddMember("Nc", json_val, allocator);

        json_val.SetArray();
        for (auto &orient : orientation) {
            tmp_json_val.SetArray();
            tmp_json_val.PushBack(orient[0], allocator)
                    .PushBack(orient[1], allocator)
                    .PushBack(orient[2], allocator);
            json_val.PushBack(tmp_json_val, allocator);
        }

        doc.AddMember("Orientation", json_val, allocator);

        json_val.SetString("END", allocator);
        doc.AddMember("Cardinal", json_val, allocator);

        json_val.SetFloat(sampling);
        doc.AddMember("Sampling", json_val, allocator);

        json_val.SetString(name.c_str(), allocator);
        doc.AddMember("Name", json_val, allocator);

        return doc;
    }

    Receiver Receiver::fromJSON(const rapidjson::Value &doc) {
        if (!doc.IsObject())
            throw std::runtime_error(
                    "Receiver::fromJSON() - document should be an object");

        std::vector<std::string> required_fields = {
                "Location", "Nc", "Orientation", "Cardinal", "Sampling", "Name"};

        for (const auto &field : required_fields) {
            if (!doc.HasMember(field.c_str()))
                throw std::runtime_error(
                        "Receiver::fromJSON() - invalid JSON, missing field " + field);
        }

        if (!doc["Location"].IsArray())
            throw std::runtime_error(
                    "Receiver::fromJSON() - invalid JSON, `Location` should be an array");

        if (doc["Location"].Size() != 3)
            throw std::runtime_error("Receiver::fromJSON - invalid JSON, wrong "
                                     "'Location' size (should be equal three)");

        if (!doc["Nc"].IsUint64())
            throw std::runtime_error("Receiver::fromJSON - invalid JSON, `Nc` should "
                                     "be an unsigned integer");
        uint64_t nc = doc["Nc"].GetUint64();

        if (!doc["Orientation"].IsArray())
            throw std::runtime_error("Receiver::fromJSON() - invalid JSON, "
                                     "`Orientation` should be an array");

        if (doc["Orientation"].Size() != nc)
            throw std::runtime_error("Receiver::fromJSON - invalid JSON, wrong "
                                     "'Orientation' size (should be equal Nc)");

        if (!doc["Cardinal"].IsString())
            throw std::runtime_error(
                    "Receiver::fromJSON() - invalid JSON, `Cardinal` should be a string");

        if (!doc["Sampling"].IsFloat())
            throw std::runtime_error(
                    "Receiver::fromJSON() - invalid JSON, `Sampling` should be a float");

        if (!doc["Name"].IsString())
            throw std::runtime_error(
                    "Receiver::fromJSON() - invalid JSON, `Name` should be a string");

        float sampling = doc["Sampling"].GetFloat();
        std::string name = doc["Name"].GetString();

        std::string cardinal = doc["Cardinal"].GetString();

        if (cardinal != "END")
            throw std::runtime_error("Receiver::fromJSON() - invalid JSON, `Cardinal` "
                                     "should be equal 'END'");

        std::array<float, 3> location{doc["Location"][0].GetFloat(),
                                      doc["Location"][1].GetFloat(),
                                      doc["Location"][2].GetFloat()};

        std::vector<std::array<float, 3>> orientation;
        for (uint64_t i = 0; i < nc; i++) {
            if (!doc["Orientation"][i].IsArray())
                throw std::runtime_error("Receiver::fromJSON() - invalid JSON, "
                                         "`Orientation` component should be an array");
            orientation.push_back({doc["Orientation"][i][0].GetFloat(),
                                   doc["Orientation"][i][1].GetFloat(),
                                   doc["Orientation"][i][2].GetFloat()});
        }

        return Receiver(location, orientation, sampling, name);
    }
} // namespace ray_tracing