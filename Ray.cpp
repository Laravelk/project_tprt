#include "Ray.hpp"
#include "Segment.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>

#include "Optimize.h"
#include "RayData.h"

#include <nlopt.h>
#include <nlopt.hpp>

typedef unsigned long ulong;

/// this namespace contains classes which worked with ray_code and trajectory
namespace ray_tracing {
void Ray::optimizeTrajectory() {
  std::vector<double> vector;

  for (unsigned long i = 1; i < trajectory.size() - 1; i++) {
    vector.push_back(trajectory.at(i)[0]);
    vector.push_back(trajectory.at(i)[1]);
  }

  RayData *ray_data = new RayData(this);

  // LD_SLSQP – 0.379634 Python + За 200 итераций результат – 0.389

  // LD_MMA - 0.385 Python +- За 70-80 итераций
  // LD_CCSAQ - 0.386 За 100-200 итераций
  // LD_AUGLAG - 0.388 150 итераций

  // LD_LBFGS terminating with uncaught exception of type
  // std::runtime_error: nlopt failure LD_TNEWTON_PRECOND_RESTART аналогично
  // LD_TNEWTON аналогично
  // LD_VAR1 аналогично

  // LN_COBYLA – OutOfMemory Python (2000) -  Эффективность сомнительная
  // LN_BOBYQA OutOfMemory при 500 итерация. Выглядит более эффективным
  // LN_NEWUOA Рассходится на определенной итерации > 1200
  // LN_PRAXIS 2000 итераций. Плохо сходится
  // LN_NELDERMEAD > 5000 итераций. Плохо сходится
  // LN_SBPLX Не меняет значение траектории вовсе
  nlopt::opt opt(nlopt::LD_AUGLAG, vector.size());
  //  std::vector<double> lb(vector.size());
  //  for (auto v : lb) {
  //    v = -1000;
  //  }
  //  double minf = 0;
  //  opt.set_lower_bounds(lb);
  opt.set_min_objective(Optimize::myfunc, ray_data);
  opt.set_xtol_abs(1e-3);
  //  opt.set_maxeval(1);
  opt.optimize(vector);
  //  minf = opt.get_maxtime();
  //  std::cout << "The result is" << std::endl;
  //  std::cout << result << std::endl;
  //  std::cout << "Minimal function value " << minf << std::endl;
  //  std::cout << "search_time: " << search_time / CLOCKS_PER_SEC << std::endl;
  //  for (auto tr : trajectory) {
  //    std::cerr << "[ " << tr.at(0) << ", " << tr.at(1) << ", " << tr.at(2)
  //              << "] " << std::endl;
  //  }

  delete ray_data;
}

/// compute ray in layer and create segments
void Ray::computeSegmentsRay() {
  auto source_location = current_source.getLocation();
  auto receiver_location = current_receiver.getLocation();

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
  //  for (auto tr : trajectory) {
  //    std::cerr << tr[0] << " " << tr[1] << " " << tr[2] << std::endl;
  //  }
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
