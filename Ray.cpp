#include "Ray.hpp"
#include "Eigen/Dense"
#include "Segment.hpp"
#include "cppoptlib/meta.h"
#include "cppoptlib/problem.h"
#include "cppoptlib/solver/bfgssolver.h"
#include <algorithm>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>

typedef unsigned long ulong;

namespace ray_tracing {
/* compute ray in layer and create segments */
void Ray::computeSegmentsRay() {
  auto source_location = source.getLocation();
  auto receiver_location = receiver.getLocation();

  // count the step
  // try create optimal tractory
  float diff_x = receiver_location[0] - source_location[0];
  float diff_y = receiver_location[1] - source_location[1];
  ulong trajectory_part_count = ray_code.size();
  float step_x = diff_x / trajectory_part_count;
  float step_y = diff_y / trajectory_part_count;
  float x = 0, y = 0, z = 0;
  x = source_location[0];
  y = source_location[1];

  trajectory.push_back({x, y, source_location[2]});
  // tpc - 1, так как receiver добавляем отдельно
  for (ulong i = 0; i < trajectory_part_count - 1; i++) {
    x += step_x;
    y += step_y;
    Horizon *hor =
        velocity_model->getLayer(ray_code.at(i).layerNumber)->getTop();
    z = hor->getDepth({x, y});
    trajectory.push_back({x, y, z});
  }
  trajectory.push_back(
      {receiver_location[0], receiver_location[1], receiver_location[2]});
}

// layer_number, up/down, wave_type
void Ray::generateCode(const std::vector<std::array<int, 3>> rayCode) {
  for (auto ray_element : rayCode) {
    Direction direction;
    WaveType type;
    if (Direction::DOWN == ray_element[1]) {
      direction = Direction::DOWN;
    } else {
      direction = Direction::UP;
    }
    if (WaveType::SWAVE == ray_element[2]) {
      type = WaveType::SWAVE;
    } else {
      type = WaveType::PWAVE;
    }
    Code code(ray_element[0], direction, type);
    ray_code.push_back(code);
  }
}

void Ray::computePath() {}

void Ray::computePathWithRayCode() {
  computeSegmentsRay();
  std::cerr << "after compute segments"
            << "\n";
  // optimizeTrajectory();
}

rapidjson::Document Ray::toJSON() {
  rapidjson::Document doc;
  /*rapidjson::Value json_val;
  rapidjson::Value tmp_json_val;
  doc.SetObject();

  auto &allocator = doc.GetAllocator();

  json_val.CopyFrom(source.toJSON(), allocator);
  doc.AddMember("Source", json_val, allocator);

  json_val.CopyFrom(receiver.toJSON(), allocator);
  doc.AddMember("Receiver", json_val, allocator);

  json_val.SetArray();
  auto trajectory = getTrajectoryP();
  for (const auto &tr_seg : trajectory) {
    tmp_json_val.SetArray();
    tmp_json_val.PushBack(tr_seg[0], allocator)
        .PushBack(tr_seg[1], allocator)
        .PushBack(tr_seg[2], allocator);
    json_val.PushBack(tmp_json_val, allocator);
  }
  doc.AddMember("TrajectoryP", json_val, allocator);

  json_val.SetArray();
  trajectory = getTrajectoryS();
  for (const auto &tr_seg : trajectory) {
    tmp_json_val.SetArray();
    tmp_json_val.PushBack(tr_seg[0], allocator)
        .PushBack(tr_seg[1], allocator)
        .PushBack(tr_seg[2], allocator);
    json_val.PushBack(tmp_json_val, allocator);
  }
  doc.AddMember("TrajectoryS", json_val, allocator);

  json_val.SetFloat(amplitudeP);
  doc.AddMember("AmplitudeP", json_val, allocator);

  json_val.SetFloat(timeP);
  doc.AddMember("TimeP", json_val, allocator);

  json_val.SetFloat(amplitudeS);
  doc.AddMember("AmplitudeS", json_val, allocator);

  json_val.SetFloat(timeS);
  doc.AddMember("TimeS", json_val, allocator);

  json_val.SetString("NONE");
  doc.AddMember("Record", json_val, allocator);
  * / return doc;
} // namespace ray_tracing

// double Ray::cost_function::f(const vnl_vector<double> &x) {
//  auto n = get_number_of_unknowns();
//  std::vector<std::array<float, 3>> trajectory;
//  std::array<float, 3> source_location;
//  double time = 0;

//  switch (type) {
//  case WaveType::WaveP:
//    trajectory = ray->getTrajectoryP();
//    for (int i = 0; i < n / 2; i++) {
//      std::array<float, 2> cord = {static_cast<float>(x[2 * i]),
//                                   static_cast<float>(x[2 * i + 1])};
//      auto t = ray->segmentsP[i].getHorizon(); // TODO: test
//      float tt = t->getDepth(cord);
//      trajectory[i + 1] = {{static_cast<float>(x[2 * i]),
//                            static_cast<float>(x[2 * i + 1]),
//                            ray->segmentsP[i].getHorizon()->getDepth(
//                                {static_cast<float>(x[2 * i]),
//                                 static_cast<float>(x[2 * i + 1])})}};
//    }
//    source_location = trajectory[0];
//    for (auto &seg : ray->segmentsP) {
//      auto receiver_location = trajectory[&seg - &ray->segmentsP[0] + 1];

//      std::array<float, 3> vec{receiver_location[0] - source_location[0],
//                               receiver_location[1] - source_location[1],
//                               receiver_location[2] - source_location[2]};

//      float norm_vec =
//          sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

//      auto layer = seg.getLayer();

//      time += norm_vec / layer.getVp();

//      source_location = receiver_location;
//    }
//    break;

//  case WaveType::WaveS:
//    trajectory = ray->getTrajectoryS(); // вместо траектории, просто изменять
//    ее
//                                        // середину, без копирования
//    for (int i = 0; i < n / 2; i++) {
//      trajectory[i + 1] = {{static_cast<float>(x[2 * i]),
//                            static_cast<float>(x[2 * i + 1]),
//                            ray->segmentsS[i].getHorizon()->getDepth(
//                                {static_cast<float>(x[2 * i]),
//                                 static_cast<float>(x[2 * i + 1])})}};
//    }
//    source_location = trajectory[0];
//    for (auto &seg : ray->segmentsS) {
//      auto receiver_location = trajectory[&seg - &ray->segmentsS[0] + 1];

//      std::array<float, 3> vec{receiver_location[0] - source_location[0],
//                               receiver_location[1] - source_location[1],
//                               receiver_location[2] - source_location[2]};

//      float norm_vec =
//          sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);

//      auto layer = seg.getLayer();

//      time += norm_vec / layer.getVs();

//      source_location = receiver_location;
//    }
//  }

//  return time;
//}

// std::vector<std::array<int, 3>>
// Ray::cost_function::fromJSON(const rapidjson::Value &doc) {
//  if (!doc.IsObject())
//    throw std::runtime_error("Ray::fromJSON() - document should be an
//    object");

//  std::vector<std::string> required_fields = {"Ray_Code"};

//  if (!doc["Ray_Code"].IsArray()) {
//    throw std::runtime_error(
//        "Ray::fromJSON() - invalid JSON, 'Array' should be a array");
//  }
//}

// Ray::cost_function::cost_function(Ray *ray, int number_of_unknowns,
//                                  enum WaveType type)
//    : ray(ray), vnl_cost_function(number_of_unknowns), type(type) {}
} // namespace ray_tracing
