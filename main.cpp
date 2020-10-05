#include "Horizon/FlatHorizon.hpp"
#include "Horizon/GridHorizon.h"
#include "Ray.hpp"
#include "VelocityModel.hpp"
#include "rapidjson/error/en.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/prettywriter.h"
#include <fstream>
#include <iostream>

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Wrong parameters number" << std::endl;
    return EXIT_FAILURE;
  }
  std::ifstream ifs{argv[1], std::ifstream::in};
  if (!ifs.is_open()) {
    std::cerr << "Could not open file for reading!\n";
    return EXIT_FAILURE;
  }

  rapidjson::IStreamWrapper isw(ifs);

  rapidjson::Document doc;
  doc.ParseStream(isw);

  if (doc.HasParseError()) {
    std::cout << "Error  : " << rapidjson::GetParseError_En(doc.GetParseError())
              << '\n'
              << "Offset : " << doc.GetErrorOffset() << '\n';
    return EXIT_FAILURE;
  }

  if (!doc.IsObject())
    throw std::runtime_error("Document should be an object");

  if (!doc.HasMember("Receiver"))
    throw std::runtime_error("Invalid JSON, missing field Receiver");
  if (!doc.HasMember("Source"))
    throw std::runtime_error("Invalid JSON, missing field Source");
  if (!doc.HasMember("Velocity model"))
    throw std::runtime_error("Invalid JSON, missing field Source");

  // auto grid_json = ray_tracing::GridHorizon::fromJSON(doc);

  // source
  auto source = ray_tracing::Source::fromJSON(doc["Source"].GetObject());
  // get info about receiver
  auto receiver = ray_tracing::Receiver::fromJSON(doc["Receiver"].GetObject());
  // get info about velocity model
  auto velocity_model =
      std::move(ray_tracing::VelocityModel::fromJSON(doc["Velocity model"]));

  // for ray_code test
  std::vector<std::array<int, 3>> ray_code;
  std::array<int, 3> f = {5, 1, 0};   // source -> 5 (5)
  std::array<int, 3> f1 = {4, 1, 0};  // 5 -> 4 (4)
  std::array<int, 3> f2 = {3, 1, 0};  // 4 -> 3 (3)
  std::array<int, 3> f3 = {2, 1, 0};  // 3 -> 2 (2)
  std::array<int, 3> f4 = {1, 1, 0};  // 2 -> 1 (1)
  std::array<int, 3> f5 = {2, -1, 0}; // 1 -> 2 (2)
  std::array<int, 3> f6 = {1, 1, 0};  // 2 -> 1 (1)
  std::array<int, 3> f7 = {0, 1, 0};  // 1 -> 0 (0)
  std::array<int, 3> f8 = {-1, 1, 0}; // 0 -> rec (rec pos)
  ray_code.push_back(f);
  ray_code.push_back(f1);
  ray_code.push_back(f2);
  ray_code.push_back(f3);
  ray_code.push_back(f4);
  ray_code.push_back(f5);
  ray_code.push_back(f6);
  ray_code.push_back(f7);
  ray_code.push_back(f8);

  ray_tracing::Ray ray(source, receiver, std::move(velocity_model), ray_code);
  ray.computePathWithRayCode();

  std::ofstream ofs(argv[2]);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  ray.toJSON().Accept(writer);
  return 0;
}
