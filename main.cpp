#include <iostream>
#include "Ray.hpp"
#include "Horizon/FlatHorizon.hpp"
#include "rapidjson/prettywriter.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/error/en.h"
#include "VelocityModel.hpp"
#include "Horizon/GridHorizon.h"
#include <fstream>

int main(int argc, char * argv[]) {
    if (argc != 3) {
        std::cerr << "Wrong parameters number" << std::endl;
        return EXIT_FAILURE;
    }
    std::ifstream ifs {argv[1], std::ifstream::in};
    if ( !ifs.is_open() )
    {
        std::cerr << "Could not open file for reading!\n";
        return EXIT_FAILURE;
    }

    rapidjson::IStreamWrapper isw(ifs);

    rapidjson::Document doc;
    doc.ParseStream(isw);

    if ( doc.HasParseError() ) {
        std::cout << "Error  : " << rapidjson::GetParseError_En(doc.GetParseError())  << '\n'
                  << "Offset : " << doc.GetErrorOffset() << '\n';
        return EXIT_FAILURE;
    }

    if(!doc.IsObject())
        throw std::runtime_error("Document should be an object");

    if(!doc.HasMember("Receiver"))
        throw std::runtime_error("Invalid JSON, missing field Receiver");
    if(!doc.HasMember("Source"))
        throw std::runtime_error("Invalid JSON, missing field Source");
    if(!doc.HasMember("Velocity model"))
        throw std::runtime_error("Invalid JSON, missing field Source");

    // source
    auto source = ray_tracing::Source::fromJSON(doc["Source"].GetObject());
    // get info about receiver
    auto receiver = ray_tracing::Receiver::fromJSON(doc["Receiver"].GetObject());
    // get info about velocity model
    auto velocity_model = ray_tracing::VelocityModel::fromJSON(doc["Velocity model"]);

    ray_tracing::Ray ray(source, receiver, velocity_model);

    auto grid_json = ray_tracing::GridHorizon::fromJSON(doc);

    ray.computePath();

    std::ofstream ofs(argv[2]);
    rapidjson::OStreamWrapper osw(ofs);
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
    ray.toJSON().Accept(writer);
    return 0;
}