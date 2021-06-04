#include <array>
#include <fstream>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>      // std::setprecision
#include <mutex>

#include "Data/Horizon/FlatHorizon.hpp"
#include "Ray/Ray.hpp"
#include "Data/VelocityModel.hpp"
#include "rapidjson/error/en.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/prettywriter.h"

#include <chrono>

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

  std::ifstream sources_file = std::ifstream("/home/laravelk/projects/project_tprt-feature-develop/Test/TestData/sources_out.txt");
  std::ifstream receivers_file= std::ifstream("/home/laravelk/projects/project_tprt-feature-develop/Test/TestData/receivers_out.txt");

  // source
//  auto sources = ray_tracing::Source::fromFile(std::move(sources_file));
  auto source = ray_tracing::Source::fromJSON(doc["Source"]);
  auto sources = ray_tracing::Source::fromFile(sources_file);
  // get info about receiver
//  auto receivers = ray_tracing::Receiver::fromFile(std::move(receivers_file));
    auto receiver = ray_tracing::Receiver::fromJSON(doc["Receiver"]);
    auto receivers = ray_tracing::Receiver::fromFile(receivers_file);

    // get info about velocity model
  auto velocity_model =
      ray_tracing::VelocityModel::fromJSON(doc["Velocity model"]);

  // create the ray
  std::vector<ray_tracing::Ray> rays;
  const long N = receivers.size();
//  const long N = 10;

//  source.change_x_loc(0);
  for (long i = 0; i < N; i++) {
      rays.emplace_back(source, receiver, velocity_model.get(), true, WaveType::PWave);
  }

  std::mutex mutex;
  int step = N / 200;
  double v = 0;
  double dur = 0;
  auto start = std::chrono::steady_clock::now();

  std::cerr << N << std::endl;
    std::ofstream out;
    out.open("/home/laravelk/projects/project_tprt-feature-develop/Test/out3.txt");

//#pragma omp parallel for schedule(guided)
    for (long i = 0; i < N; i++) {
        rays[i].computePathWithRayCode();
//        mutex.lock();
//        v++;
//        out << i << ": " << std::setprecision(10) << rays[i].getTrajectory()[4][0] << ", " << rays[i].getTrajectory()[4][1] << ", " << rays[i].getTrajectory()[4][2] << ", " << rays[i].traveltime << std::endl;
//        if (v > dur) {
//            dur += step;
//            std::cerr << v / (double)N * 100 << "% " << v << " " <<  i << std::endl;
//        }
//        mutex.unlock();

        auto spreading = rays[i].raySpreading();
    }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

  if (out.is_open()) {
      for (int i = 0; i < rays.size(); i++) {
          out << i << ": " << std::setprecision(10) << rays[i].getTrajectory()[4][0] << ", " << rays[i].getTrajectory()[4][1] << ", " << rays[i].getTrajectory()[4][2] << ", " << rays[i].traveltime << std::endl;
      }
  }

  out.close();

  // compute path with ray code

  std::ofstream ofs(argv2);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
  //  ray.toJSON().Accept(writer);
  return 0;
}
