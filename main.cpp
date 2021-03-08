#include <array>
#include <fstream>
#include <iostream>
#include <vector>

#include "Horizon/FlatHorizon.hpp"
#include "Horizon/GridHorizon.h"
#include "Ray/Ray.hpp"
#include "VelocityModel.hpp"
#include "rapidjson/error/en.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/prettywriter.h"

#include <chrono>

std::vector<std::array<int, 3>> getRayCode2() {
  std::vector<std::array<int, 3>> ray_code;
  std::array<int, 3> f = {1, -1, 0};
  std::array<int, 3> f1 = {2, -1, 0};
  std::array<int, 3> f2 = {3, -1, 0};
  std::array<int, 3> f3 = {4, -1, 0};
  std::array<int, 3> f4 = {5, -1, 0};

  ray_code.push_back(f);
  ray_code.push_back(f1);
  ray_code.push_back(f2);
  ray_code.push_back(f3);
  ray_code.push_back(f4);

  return ray_code;
}

std::vector<std::array<int, 3>> getRayCode4() {
  std::vector<std::array<int, 3>> ray_code;
  std::array<int, 3> f = {1, -1, 0};
  std::array<int, 3> f1 = {1, 1, 0};

  ray_code.push_back(f);
  ray_code.push_back(f1);
  return ray_code;
}

std::vector<std::array<int, 3>> getRayCode1() {
  std::vector<std::array<int, 3>> ray_code;
  std::array<int, 3> f = {5, 1, 0};   // source -> 5 (5)
  std::array<int, 3> f1 = {4, 1, 0};  // 5 -> 4 (4)
  std::array<int, 3> f2 = {3, 1, 0};  // 4 -> 3 (3)
  std::array<int, 3> f3 = {2, 1, 0};  // 3 -> 2 (2)
  std::array<int, 3> f4 = {1, 1, 0};  // 2 -> 1 (1)
  std::array<int, 3> f5 = {1, -1, 0}; // 1 -> 2 (2)
  std::array<int, 3> f6 = {1, 1, 0};  // 2 -> 1 (1)
  std::array<int, 3> f7 = {0, 1,
                           0}; // 1 -> rec (rec pos): it's under zero layer
  ray_code.push_back(f);
  ray_code.push_back(f1);
  ray_code.push_back(f2);
  ray_code.push_back(f3);
  ray_code.push_back(f4);
  ray_code.push_back(f5);
  ray_code.push_back(f6);
  ray_code.push_back(f7);
  return ray_code;
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Wrong parameters number" << std::endl;
    return EXIT_FAILURE;
  }

  std::string argv1 = argv[1];
  std::string argv2 = argv[2];

  std::ifstream ifs{argv1, std::ifstream::in};
  if (!ifs.is_open()) {
    std::cerr << "Could not open file for reading! Your file name is " + argv1 +
                     "\n";
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

  std::ifstream sources_file("sources.txt");
  std::ifstream receivers_file("receivers.txt");

  // source
  auto sources = ray_tracing::Source::fromFile(std::move(sources_file));
  // get info about receiver
  auto receivers = ray_tracing::Receiver::fromFile(std::move(receivers_file));
  // get info about velocity model
  auto velocity_model =
      ray_tracing::VelocityModel::fromJSON(doc["Velocity model"]);

  // for ray code test
  std::vector<std::array<int, 3>> ray_code = getRayCode4();
  // create the ray
  std::vector<ray_tracing::Ray> rays;
  const long N = 100;

  for (long i = 0; i < N; i++) {
    rays.emplace_back(sources[i], receivers[i], velocity_model.get(), ray_code);
  }

  auto start = std::chrono::steady_clock::now();
  for (long i = 0; i < N; i++) {
    rays[i].computePathWithRayCode();
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  // compute path with ray code

  std::ofstream ofs(argv2);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  //  ray.toJSON().Accept(writer);
  return 0;
}
