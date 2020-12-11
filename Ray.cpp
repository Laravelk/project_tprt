#include "Ray.hpp"
#include "Segment.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>

#include <nlopt.h>
#include <nlopt.hpp>

// check module -- проверяет, что слои все хорошо

typedef unsigned long ulong;

/// this namespace contains classes which worked with ray_code and trajectory
namespace ray_tracing {

void Ray::optimizeTrajectory() {
  std::vector<double> vector;

  for (int i = 1; i < trajectory.size() - 1; i++) {
    vector.push_back(trajectory.at(i)[0]);
    vector.push_back(trajectory.at(i)[1]);
  }

  nlopt::opt opt(nlopt::LD_LBFGS, vector.size());
  opt.set_min_objective(myfunc, NULL);
  opt.optimize(vector);

  for (auto tr : trajectory) {
    std::cerr << "[ " << tr.at(0) << ", " << tr.at(1) << ", " << tr.at(2)
              << "] " << std::endl;
  }
}

/// compute ray in layer and create segments
void Ray::computeSegmentsRay() {
  auto source_location = source.getLocation();
  auto receiver_location = receiver.getLocation();

  float diff_x = receiver_location[0] - source_location[0];
  float diff_y = receiver_location[1] - source_location[1];
  ulong trajectory_part_count = ray_code.size();
  float step_x = diff_x / trajectory_part_count;
  float step_y = diff_y / trajectory_part_count;
  float x = 0, y = 0, z = 0;
  x = source_location[0];
  y = source_location[1];

  trajectory.push_back({x, y, source_location[2]});
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

double Ray::calculate_part_time(const std::array<float, 3> source,
                                const std::array<float, 3> receiver,
                                int layer_number) {
  std::array<double, 3> vec{receiver[0] - source[0], receiver[1] - source[1],
                            receiver[2] - source[2]};

  double norm_vec = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
  auto layer = velocity_model->getLayer(layer_number);
  double vp = static_cast<double>(layer->getVp());

  //  std::cerr << "norm_vec / vp " << norm_vec << " " << vp << std::endl;

  return norm_vec / vp;
}

double Ray::calculate_full_time(const std::vector<double> &x) {
  double time = 0;
  int number_of_unknowns = trajectory.size();
  for (int i = 1; i <= number_of_unknowns - 2; i++) {
    trajectory[i] = {static_cast<float>(x[2 * (i - 1)]),
                     static_cast<float>(x[2 * (i - 1) + 1]),
                     velocity_model->getLayer(ray_code[i - 1].layerNumber)
                         ->getTop()
                         ->getDepth({static_cast<float>(x[2 * (i - 1)]),
                                     static_cast<float>(x[2 * (i - 1) + 1])})};
  }

  std::array<float, 3> source_location = source.getLocation();

  for (int i = 1; i < number_of_unknowns - 1; i++) {
    std::array<float, 3> receiver_location = trajectory.at(i);
    double new_time = calculate_part_time(source_location, receiver_location,
                                          ray_code.at(i - 1).layerNumber - 1);
    time += new_time;
    source_location = receiver_location;
  }

  int lastLayer = 0;
  if (Direction::DOWN == ray_code.at(ray_code.size() - 2).direction) {
    lastLayer = velocity_model->getLayersCount() - 1;
  } else {
    lastLayer = 0;
  }

  time +=
      calculate_part_time(source_location, receiver.getLocation(), lastLayer);

  std::cerr << time << std::endl;

  return time;
}

double Ray::myfunc(const std::vector<double> &x, std::vector<double> &grad,
                   void *my_func_data) {
  if (!grad.empty()) {
    float xk_1 = trajectory[0][0]; // xk-1
    float yk_1 = trajectory[0][1];
    float zk_1 = trajectory[0][2];
    unsigned long number_of_unknown = trajectory.size();

    //    for (int i = 0; i < x.size(); i++) {
    //      std::cerr << x[i] << " ";
    //    }
    //    std::cerr << std::endl;

    for (long i = 0; i < number_of_unknown - 2; i++) {
      float xk = trajectory[i + 1][0];
      float yk = trajectory[i + 1][1];
      float zk = trajectory[i + 1][2];

      float xk_plus_1 = trajectory[i + 2][0];
      float yk_plus_1 = trajectory[i + 2][1];
      float zk_plus_1 = trajectory[i + 2][2];

      std::array<double, 2> derive =
          velocity_model->getLayer(ray_code[i].layerNumber)
              ->getTop()
              ->getGradientInPoint(xk, yk);

      float DzkDxk = (float)derive[0]; // dzk/dxk
      float DzkDyk = (float)derive[1];

      //      std::cerr << "DzkDxk: " << DzkDxk << std::endl;
      //      std::cerr << "DzkDyk: " << DzkDyk << std::endl;

      float Dvk_1Dxk = 0; // isotropic envoriment
      float Dvk_1Dyk = 0; // isotropic envoriment

      float DvkDxk_plus_1 = 0; // isotropic envoriment
      float DvkDyk_plus_1 = 0; // isotropic envoriment

      float vk =
          (float)(velocity_model->getLayer(ray_code[i].layerNumber)
                      ->Vp); // vk
                             //      std::cerr << "vk: " << vk << std::endl;
      float vk_1 = 0;
      int expression = i - 1;
      if (expression < 0) {
        vk_1 = (float)velocity_model->getLayer(0)->Vp;
      } else {
        vk_1 = (float)velocity_model->getLayer(ray_code[i - 1].layerNumber)
                   ->Vp; // vk-1
      }
      //      std::cerr << "vk_1: " << vk_1 << std::endl;

      float lk = sqrt((xk_plus_1 - xk) * (xk_plus_1 - xk) +
                      (yk_plus_1 - yk) * (yk_plus_1 - yk) +
                      (zk_plus_1 - zk) * (zk_plus_1 - zk));
      //      std::cerr << "lk: " << lk << std::endl;

      float lk_1 = sqrt((xk - xk_1) * (xk - xk_1) + (yk - yk_1) * (yk - yk_1) +
                        (zk - zk_1) * (zk - zk_1)); // lk-1
      //      std::cerr << "lk_1: " << lk_1 << std::endl;

      float DtDxk = (xk - xk_1 + (zk - zk_1) * DzkDxk) / (lk_1 * vk_1) -
                    (xk_plus_1 - xk + (zk_plus_1 - zk) * DzkDxk) / (lk * vk);

      //      std::cerr << "DtDxk: " << DtDxk << std::endl;

      float DtDyk = (yk - yk_1 + (zk - zk_1) * DzkDyk) / (lk_1 * vk_1) -
                    (yk_plus_1 - yk + (zk_plus_1 - zk) * DzkDyk) / (lk * vk);

      //      std::cerr << "DtDyk: " << DtDyk << std::endl << std::endl;

      grad[2 * i] = DtDxk;
      grad[2 * i + 1] = DtDyk;

      xk_1 = xk;
      yk_1 = yk;
      zk_1 = zk;
    }
  }
  return calculate_full_time(x);
}

void Ray::computePath() {}

void Ray::computePathWithRayCode() {
  computeSegmentsRay();
  optimizeTrajectory();
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
  */
  return doc;
} // namespace ray_tracing

} // namespace ray_tracing
