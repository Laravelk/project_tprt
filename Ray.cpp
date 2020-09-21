#include "Ray.hpp"
#include "Eigen/Dense"
#include "Segment.hpp"
#include <algorithm>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>
#include <vnl/algo/vnl_amoeba.h>
#include <vnl/vnl_cost_function.h>

typedef unsigned long ulong;

namespace ray_tracing {
/* compute ray in layer and create segments */
/*void Ray::computeSegments() {
  std::vector<std::array<float, 3>> intersections;

  auto source_location = source.getLocation();
  auto receiver_location = receiver.getLocation();

  //count the step
// try create optimal tractory
float diff_x = receiver_location[0] - source_location[0];
float diff_y = receiver_location[1] - source_location[1];
float layers_count = velocity_model.getLayers().size();
float step_x = diff_x / layers_count;
float step_y = diff_y / layers_count;
float x = 0, y = 0, z = 0;
x = source_location[0];
y = source_location[1];

auto layers = velocity_model.getLayers();
for (ulong i = 0; i < layers_count; i++) {
  x += step_x;
  y += step_y;
  std::array<float, 2> coordinates = {x, y};
  z = layers.at(layers_count - i - 1).getTop()->getDepth(coordinates);
  std::array<float, 3> intersect = {x, y, z};
  intersections.push_back(intersect);
}

auto loc_source_location = source.getLocation();
ulong layer_number = 0;
for (auto &loc_receiver_location : intersections) {
  std::array<float, 3> vec{loc_receiver_location[0] - loc_source_location[0],
                           loc_receiver_location[1] - loc_source_location[1],
                           loc_receiver_location[2] - loc_source_location[2]};
  float norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
  vec[0] /= norm_vec;
  vec[1] /= norm_vec;
  vec[2] /= norm_vec;

  auto layer = getLocationLayer({loc_source_location[0] + vec[0],
                                 loc_source_location[1] + vec[1],
                                 loc_source_location[2] + vec[2]});
  layer_number++;
  loc_source_location = loc_receiver_location;
}
} // namespace ray_tracing */

void Ray::computeSegments() {
  auto source_location = source.getLocation();
  auto receiver_location = receiver.getLocation();

  // count the step
  // try create optimal tractory
  float diff_x = receiver_location[0] - source_location[0];
  float diff_y = receiver_location[1] - source_location[1];
  float trajectory_part_count = ray_code.size();
  float step_x = diff_x / trajectory_part_count;
  float step_y = diff_y / trajectory_part_count;
  float x = 0, y = 0, z = 0;
  x = source_location[0];
  y = source_location[1];

  auto layers = velocity_model.getLayers();
  trajectory.push_back({x, y, source_location[2]});
  for (auto code : ray_code) {
    x += step_x;
    y += step_y;
    z = layers.at(code.layerNumber).getTop()->getDepth({x, y});
    trajectory.push_back({x, y, z});
  }
  trajectory.push_back(
      {receiver_location[0], receiver_location[1], receiver_location[2]});
}

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

  auto layers = velocity_model.getLayers();
  trajectory.push_back({x, y, source_location[2]});
  // tpc - 1, так как receiver добавляем отдельно
  for (ulong i = 0; i < trajectory_part_count - 1; i++) {
    x += step_x;
    y += step_y;
    z = layers.at(ray_code.at(i).layerNumber - 1).getTop()->getDepth({x, y});
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

/* @return min layer's position */
Layer Ray::getLocationLayer(std::array<float, 3> location) {
  std::vector<Layer> higher = velocity_model.getLayers();
  std::copy_if(
      velocity_model.getLayers().begin(), velocity_model.getLayers().end(),
      std::back_inserter(higher), [&location](Layer layer) {
        auto loc = layer.getTop()->getDepth({location[0], location[1]});
        auto result = loc > location[2];
        return result;
      });
  auto it = std::min_element(
      higher.begin(), higher.end(), [&location](Layer &a, Layer &b) {
        return a.getTop()->getDepth({location[0], location[1]}) - location[3] >
               b.getTop()->getDepth({location[0], location[1]}) - location[3];
      });
  return *it;
}

// std::vector<std::array<float, 3>> Ray::getTrajectoryP() {
//  std::vector<std::array<float, 3>> trajectory;
//  trajectory.push_back(source.getLocation());
//  for (auto &seg : segmentsP) {
//    trajectory.push_back(seg.getReceiver_location());
//  }
//  return trajectory;
//}

// std::vector<std::array<float, 3>> Ray::getTrajectoryS() {
//  std::vector<std::array<float, 3>> trajectory;
//  trajectory.push_back(source.getLocation());
//  for (auto &seg : segmentsS) {
//    trajectory.push_back(seg.getReceiver_location());
//  }
//  return trajectory;
//}

void Ray::computePath() {
  // std::cerr << "Ray::computePath()" << std::endl; // TODO: delete
  computeSegments();
  // optimizeTrajectory();
}

void Ray::computePathWithRayCode() {
  computeSegmentsRay();
  std::cerr << "after compute segments"
            << "\n";
  // optimizeTrajectory();
}

/*void Ray::optimizeTrajectory() {
  auto trajectory = getTrajectoryP();

  auto number_of_unknowns = (trajectory.size() - 2) * 2;
  vnl_vector<double> x(number_of_unknowns);

  for (unsigned long i = 0; i < number_of_unknowns / 2; i++) {
    x[2 * i] = trajectory[i + 1][0];
    x[2 * i + 1] = trajectory[i + 1][1];
  }

  std::cerr << "function" << std::endl;
  cost_function function(this, static_cast<int>(number_of_unknowns),
                         WaveType::WaveP);
  std::cerr << "after" << std::endl;

  vnl_amoeba solver(function);
  std::cerr << "solver" << std::endl;
  solver.minimize(x);
  std::cerr << "after solver" << std::endl;

  for (int i = 0; i < number_of_unknowns / 2; i++) {
    segmentsP[i].setReceiver_location(
        {{static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
          segmentsP[i].getHorizon()->getDepth(
              {static_cast<float>(x[2 * i]),
               static_cast<float>(x[2 * i + 1])})}});
  }
  timeP = static_cast<float>(function.f(x));

  trajectory = getTrajectoryS();
  for (unsigned long i = 0; i < number_of_unknowns / 2; i++) {
    x[2 * i] = trajectory[i + 1][0];
    x[2 * i + 1] = trajectory[i + 1][1];
  }
  cost_function functionS(this, static_cast<int>(number_of_unknowns),
                          WaveType::WaveS);

  vnl_amoeba solverS(functionS);
  solverS.minimize(x);
  for (int i = 0; i < number_of_unknowns / 2; i++) {
    segmentsS[i].setReceiver_location(
        {{static_cast<float>(x[2 * i]), static_cast<float>(x[2 * i + 1]),
          segmentsS[i].getHorizon()->getDepth(
              {static_cast<float>(x[2 * i]),
               static_cast<float>(x[2 * i + 1])})}});
  }

  timeS = static_cast<float>(functionS.f(x));
}*/

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
*/
  return doc;
}

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
